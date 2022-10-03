/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * <Vasileios Anagnostoulis >
 * <Koromilas Panagiotis>
 *
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;

/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	WARN_ON ( !(sensor = state->sensor));
	if (sensor->msr_data[state->type]->last_update > state->buf_timestamp)
		return 1; //needs refresh 
	else
		return 0; // no refresh needed
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */

static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	uint32_t value_read, timestamp; 
	long lookup_value;
	long akeraio, dekadiko;
	char prosimo;
	sensor = state->sensor;
	WARN_ON(!sensor);
	debug("leaving\n");

	/*
	* Update the raw values and the relevant timestamps.
	* Grab the raw data quickly, hold the
	* spinlock for as little as possible.
	*/
	// idio spinlock gia write kai read
	// write ginetai se itnerupt conext
	spin_lock(&sensor->lock);
	value_read = sensor->msr_data[state->type]->values[0];
	timestamp = sensor->msr_data[state->type]->last_update;
	spin_unlock(&sensor->lock);

	if (!lunix_chrdev_state_needs_refresh(state))
		return -EAGAIN;
	else
	{
		debug("Time to process the data from lookup table\n");
		if (state->type == 0)
			lookup_value = lookup_voltage[value_read];
		else if (state->type == 1)
			lookup_value = lookup_temperature[value_read];
		 else if (state->type == 2)
		 	lookup_value = lookup_light[value_read];
		else
			return -EFAULT;
		// PROCESS THEM
		prosimo='+';
		if (lookup_value < 0)
		{
			lookup_value= -lookup_value;
			prosimo='-';
		}
		akeraio = lookup_value / 1000;
		dekadiko = lookup_value % 1000;
		state->buf_lim = snprintf(state->buf_data, LUNIX_CHRDEV_BUFSZ, "%c%ld.%03ld\n", prosimo, akeraio, dekadiko);
		state->buf_timestamp= timestamp;
		return 0;
	}
}
/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	/* Declarations */
	int ret;
	unsigned int minor_n;
	minor_n = iminor(inode);
	struct lunix_chrdev_state_struct* state;

    debug("opening\n");
	ret = -ENODEV;
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto out;

	// allocate space for lunix_chrdev_state_struct
	// process context that may sleep so GFP_KERNEL
	state = kmalloc(sizeof (struct lunix_chrdev_state_struct), GFP_KERNEL);
	if (!state){
		debug("kmalloc failed\n");
		ret = -ENOMEM;
		goto out;
	}
	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */
	// type of the device: Batt = 0 , light, temp
	state->type = minor_n % 8;
	if (state->type < 0 || state->type > 2) {
		debug("Not proper device measurement. The number is %d.\n", state->type);
		goto out;
	}
	// initialize buf lim,timestamp and semaphore
	state->buf_lim = 0;
	state->buf_timestamp = 0;
	sema_init(&(state->lock), 1);
	/* Allocate a new Lunix character device private state structure */
	state->sensor = &lunix_sensors[minor_n/8];
	filp-> private_data = state;

out:
	debug("leaving, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	kfree(filp->private_data);
	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	/* Why? */
	// bc it is not supported?
	return -EINVAL;
}

// cnt =  are the requested data
// filp = file pointer
// f_pos = pointer ston buf_data 
// usrbuff = USERSPACE POINTE the user buffer holding the data to be written or the empty buffer where the newly read data should be placed.
static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	//f_pos= pointer sto buff data, einai to offset tou buff data
	ssize_t ret;
	int count;
	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;
	state = filp->private_data;
	WARN_ON(!state);
	sensor = state->sensor;
	WARN_ON(!sensor);

	//	lock to read in case of read from two different procs on the same chdev_state
	if (down_interruptible(&state->lock))
 		return -ERESTARTSYS;
	debug("READ START\n");
	// if i start to read now from buffer
	if (*f_pos == 0) {
		// 	EAGAIN there is no data available right now, try again later
		while (lunix_chrdev_state_update(state) == -EAGAIN) {
			// no data release the lock
			up(&state->lock); 
			debug("reading: going to sleep\n");
			if (wait_event_interruptible(sensor->wq, lunix_chrdev_state_needs_refresh(state)))
				return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
			/* otherwise loop, but first reacquire the lock */
			if (down_interruptible(&state->lock))
				return -ERESTARTSYS;
		}
	}
	/* End of file */
	/* ? */
	if (state->buf_lim == 0) {
        ret = 0;
        goto out;
	}
	
	
	/* Determine the number of cached bytes to copy to userspace */
	/* ? */
	if((state->buf_lim - *f_pos) >= cnt)
		count = cnt;
	else 
		count = state->buf_lim - *f_pos;
	
	if(copy_to_user(usrbuf, state->buf_data + *f_pos ,count)){
		ret = -EFAULT;
		goto out;
	}
	*f_pos += count;
	if(*f_pos >= state->buf_lim){
		*f_pos = 0;
	}
	ret = count;
	
	
out:
	/* Unlock? */
	up(&state->lock);
	return ret;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct file_operations lunix_chrdev_fops = 
{
    .owner          = THIS_MODULE,
	.open           = lunix_chrdev_open,
	.release        = lunix_chrdev_release,
	.read           = lunix_chrdev_read,
	.unlocked_ioctl = lunix_chrdev_ioctl,
	.mmap           = lunix_chrdev_mmap
};

int lunix_chrdev_init(void)
{
	/*
	 * Register the character device with the kernel, asking for
	 * a range of minor numbers (number of sensors * 8 measurements / sensor)
	 * beginning with LINUX_CHRDEV_MAJOR:0
	 */
	int ret;
	dev_t dev_no;

	// lunix_sensor_cnt is defined 16
	// minor = 16 * 8 == cnt << 3
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;
	
	debug("initializing character device\n");
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;

	// LUNIX_CHRDEV_MAJOR defined 60
	// this is held in first 12 bits of dev_t
	// and the rest 20 are for the minor number
	// MAJOR (dev_t dev_no) to obtain a major from dev_t

	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	// major=60 and minor= 0 is obrained in dev_no 
	ret = register_chrdev_region (dev_no, lunix_minor_cnt , "lunix-tng");
	/* register_chrdev_region? */
	if (ret < 0) {
		debug("failed to register region, ret = %d\n", ret);
		goto out;
	}	

	ret = cdev_add(&lunix_chrdev_cdev , dev_no ,lunix_minor_cnt);
	if (ret < 0) {
		debug("failed to add character device\n");
		goto out_with_chrdev_region;
	}

	// ret < 0 means problem!
	debug("completed successfully\n");
	return 0;

out_with_chrdev_region:
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
	return ret;
}

void lunix_chrdev_destroy(void)
{ 
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;
	debug("entering\n");
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	cdev_del(&lunix_chrdev_cdev);
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	debug("leaving\n");
}
