#ifndef __OF_PCI_H
#define __OF_PCI_H

#include <linux/pci.h>

struct pci_dev;
struct of_irq;
int of_irq_map_pci(const struct pci_dev *pdev, struct of_irq *out_irq);
int of_irq_parse_and_map_pci(const struct pci_dev *dev, u8 slot, u8 pin);

struct device_node;
struct device_node *of_pci_find_child_device(struct device_node *parent,
					     unsigned int devfn);
int of_pci_parse_bus_range(struct device_node *node, struct resource *res);

#endif
