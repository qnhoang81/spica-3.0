#ifndef __LINUX_INSTINCTQ_BT_H
#define __LINUX_INSTINCTQ_BT_H

struct instinctq_bt_pdata {
	unsigned int	gpio_host_wake;

	void		(*set_power)(int);
};

#endif /* __LINUX_INSTINCTQ_BT_H */
