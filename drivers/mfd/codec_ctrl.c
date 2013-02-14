#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/device.h>

struct cdc_ctrl_dev{
	char * name;
	struct device * dev;
	struct cdev * cdev;
};

struct cdc_ctrl_dev *codec_ctrl_dev;

struct reg_param{
	int addr;
	int  val;
};

#define CODEC_SET_REGISTER 0x123
#define CODEC_GET_REGISTER 0x124

static struct class * cdcctrl_class;

int cdc_ctrl_open(struct inode * inode, struct file * filp){
	struct cdc_ctrl_dev * dev = NULL;
	
        dev = container_of(&(inode->i_cdev), struct cdc_ctrl_dev, cdev);
//	filp->private_data=(void*)dev;

	printk(KERN_ERR"[audtool] codec controler opened!\n");

	return 0;
}

int cdc_ctrl_release(struct inode *inode, struct file *filp){

	//struct cdc_ctrl_dev *dev = (struct cdc_ctrl_dev *)filp->private_data;
	
	printk(KERN_ERR"[audtool] codec controler closed!\n");
	return 0;
}

extern int adie_codec_debug_write(u8 reg, u8 mask, u8 val);
extern int adie_codec_debug_read(u8 reg,u8 *val);

long cdc_ctrl_ioctl (struct file *filp, unsigned int cmd, unsigned long arg){
	
	//struct cdc_ctrl_dev *dev = (struct cdc_ctrl_dev *)filp->private_data;
	struct reg_param * regp = kmalloc(sizeof(struct reg_param),GFP_KERNEL);
	
	unsigned char addr = 0;
	unsigned char val  = 0;

	//printk(KERN_ERR"[audtool] setting the codec register: addr:%x val:%x\n",addr,val);
        if(copy_from_user(regp,(const void *)arg, sizeof(struct reg_param))){
		kfree(regp);
		return -EFAULT;
	}
	printk(KERN_ERR"[audtool] addr:%x val:%x\n",regp->addr,regp->val);

	switch(cmd){
	case CODEC_SET_REGISTER:
		if(regp->addr<=0xff && regp->val<= 0xff){
			addr = (unsigned char)regp->addr;
			val = (unsigned char)regp->val;
			adie_codec_debug_write(addr,0xff,val);
		}else{
			printk(KERN_ERR"[audtool] illegal register address or val\n");
			return -EINVAL;
		}
		break;
        case CODEC_GET_REGISTER:
		if(regp->addr <= 0xff){
			addr = (unsigned char)regp->addr;
			adie_codec_debug_read(addr,&val);
			printk(KERN_ERR"[audtool] GETREG addr:%x val:%x",addr,val);
		}else{
			printk(KERN_ERR"[audtool] illegal register address or val\n");
			return -EINVAL;
		}

		regp->val = val;
		if(copy_to_user((void*)arg, regp, sizeof(struct reg_param))){
			printk(KERN_ERR"fail to copy regval to user space");
			return -EFAULT;
		}
		break;
	default:
		printk(KERN_ERR"[audtool] illegal cmd for audtool driver\n");
		break;
	}
	return 0;
}








static dev_t cdc_ctrl_devno;

struct file_operations cdc_ctrl_ops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = cdc_ctrl_ioctl,
	.open  = cdc_ctrl_open,
	.release = cdc_ctrl_release,
};

struct cdc_ctrl_info
{
	char * name;
};

struct cdc_ctrl_info cdcctrl_info[1]={
	{
            .name = "codec_ctrl",
	}
	
};


static void
codec_ctrl_create(struct cdc_ctrl_dev* cdcdev,char* name,struct device* parent,dev_t devt)
{
	int err;
	struct device * dev;
	
	dev = device_create(cdcctrl_class, parent, devt, NULL, "%s", name);
	
	codec_ctrl_dev->cdev = cdev_alloc();
	cdev_init(codec_ctrl_dev->cdev, &cdc_ctrl_ops);
	codec_ctrl_dev->cdev->owner = THIS_MODULE;
	
	err = cdev_add(codec_ctrl_dev->cdev, cdc_ctrl_devno, 1);
	if(err){
		printk(KERN_ERR"Error %d addind codec ctrl device",err);
		device_destroy(cdcctrl_class, devt);
	}else{
		codec_ctrl_dev->dev = dev;
		codec_ctrl_dev->name = name;
	}
}



static int 
__init codec_ctrl_init(void){
	
	int rc;
	
	codec_ctrl_dev = kzalloc(sizeof(struct cdc_ctrl_dev ),GFP_KERNEL);
	if(!codec_ctrl_dev)
		return -ENOMEM;
	
	cdcctrl_class = class_create(THIS_MODULE, "codec_ctrl");
        if (IS_ERR(cdcctrl_class)){
		kfree(codec_ctrl_dev);
		return -ENOMEM;
	}

	rc = alloc_chrdev_region(&cdc_ctrl_devno, 0, 1, "cdc_ctrl");
	if(rc < 0){
		class_unregister(cdcctrl_class);
		kfree(codec_ctrl_dev);
		return rc;
	}

	codec_ctrl_create(codec_ctrl_dev,
			  cdcctrl_info[0].name,NULL,
                          MKDEV(MAJOR(cdc_ctrl_devno),0));
	
	return 0;        
}	


static void
__exit codec_ctrl_exit(void){
	cdev_del(codec_ctrl_dev->cdev);
	kfree(codec_ctrl_dev);
}

module_init(codec_ctrl_init);
module_exit(codec_ctrl_exit);
