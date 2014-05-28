#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
 
#include <linux/interrupt.h>
#include <linux/gpio.h>
 
 
// The GPIO_17 is usually not used (pin 11 on P5 pinout raspberry pi rev. 2 
// board), we will use this to generate interrupt (rising edge). Refer  to
// the raspberry pi spec sheets.
#define GPIO_INT_GPIO17                17
 
// We need to define what string will be displayed in /proc/interrupt. 
// You may want to display the number of times our interrupt is called
// via cat /proc/interrupt
#define GPIO_INT_GPIO17_DESC           "GPIO_17 Interrupt switch"
 
// String below is not needed for now, it's needed for more complex cases. We
// can ignore this for now.
#define GPIO_INT_GPIO17_DEVICE_DESC    "GPIO17"
 
 
/****************************************************************************/
/* Interrupts variables block                                               */
/****************************************************************************/
short int irq_any_gpio    = 0;

/****************************************************************************/
/* Tasklets - provides the actual processing of the IRQ using the           */
/* bottom half approach (lower priority handlers). These are non re-entrant */
/* handlers (different from sofirqs). This is where we are going to set the */
/* processing of our leds (actually just output something to leds).         */
/****************************************************************************/

/* Forward declaration of the led_output_hander, we use this
 * for declaring a tasklet below.
 */

static void led_output_handler(unsigned long data);

DECLARE_TASKLET(ledtasklet, led_output_handler, 0L);

static void led_output_handler(unsigned long data)
{
   // We need to make sure that our tasklet is not scheduled again
   tasklet_disable(&ledtasklet);
	
   printk("%s: Takslets executed!\n", __FUNCTION__);
   
   // TODO: Set the led buttons here

   tasklet_enable(&ledtasklet);   
}


/****************************************************************************/
/* IRQ handler                                                              */
/****************************************************************************/
static irqreturn_t r_irq_handler(int irq, void *dev_id, struct pt_regs *regs) 
{
 
   unsigned long flags;
   
   // disable hard interrupts (remember them in flag 'flags')
   local_irq_save(flags);
   
   // We defer handling of our IRQ to tasklets
   tasklet_schedule(&ledtasklet);
 
   // restore hard interrupts
   local_irq_restore(flags);
 
   return IRQ_HANDLED;
}
 
 
/****************************************************************************/
/* This is our GPIO initialization function for configuring GPIO17 as our   */
/* interrupt source.                                                        */
/****************************************************************************/
void r_int_config(void) 
{
	// We first need to request the GPIO base that we require GPIO17
	// to be exported or made available.   
	if (gpio_request(GPIO_INT_GPIO17, GPIO_INT_GPIO17_DESC)) {
		printk("%s: GPIO request faiure: %s\n", __FUNCTION__, GPIO_INT_GPIO17_DESC);
		return;
	}
	// After a successful request, we need to instruct the kernel that this
	// pin will be used as an input source.
	if (gpio_direction_input(GPIO_INT_GPIO17) < 0)
	{
		printk("%s: Error setting GPIO direction!\n", __FUNCTION__);
		return;
	}
	
	if ( (irq_any_gpio = gpio_to_irq(GPIO_INT_GPIO17)) < 0 ) 
	{
		printk("%s: GPIO to IRQ mapping failure %s\n", __FUNCTION__, GPIO_INT_GPIO17_DESC);
		return;
	}
 
   printk("%s: Mapped interrupt %d\n", __FUNCTION__, irq_any_gpio);
 
   if (request_irq(irq_any_gpio,
                   (irq_handler_t ) r_irq_handler,
                   IRQF_TRIGGER_RISING,
                   GPIO_INT_GPIO17_DESC,
                   GPIO_INT_GPIO17_DEVICE_DESC)) 
	{
		printk("%s: Irq Request failure\n", __FUNCTION__);
		return;
	}
 
   return;
}
 
 
/****************************************************************************/
/* This function releases interrupts.                                       */
/****************************************************************************/
void r_int_release(void) {
 
   free_irq(irq_any_gpio, GPIO_INT_GPIO17_DEVICE_DESC);
   gpio_free(GPIO_INT_GPIO17);
 
   return;
}
 
 
/****************************************************************************/
/* Module init / cleanup block.                                             */
/****************************************************************************/
int r_init(void) {
 
   printk(KERN_NOTICE "Hello !\n");
   r_int_config();
 
   return 0;
}
 
void r_cleanup(void) {
   printk(KERN_NOTICE "Goodbye\n");
   r_int_release();
 
   return;
}
 
 
module_init(r_init);
module_exit(r_cleanup);
 
 
/****************************************************************************/
/* Module licensing/description block.                                      */
/****************************************************************************/
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vergil Cola <vpcola@gmail.com>");
MODULE_DESCRIPTION("Sample GPIO Interrupt Handler");
