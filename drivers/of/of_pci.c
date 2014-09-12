#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/of.h>
#include <linux/of_pci.h>
#include <asm/prom.h>

#include "of_private.h"

static inline int __of_pci_pci_compare(struct device_node *node,
				       unsigned int devfn)
{
	unsigned int size;
	const __be32 *reg = of_get_property(node, "reg", &size);

	if (!reg || size < 5 * sizeof(__be32))
		return 0;
	return ((be32_to_cpup(&reg[0]) >> 8) & 0xff) == devfn;
}

struct device_node *of_pci_find_child_device(struct device_node *parent,
					     unsigned int devfn)
{
	struct device_node *node, *node2;

	for_each_child_of_node(parent, node) {
		if (__of_pci_pci_compare(node, devfn))
			return node;
		/*
		 * Some OFs create a parent node "multifunc-device" as
		 * a fake root for all functions of a multi-function
		 * device we go down them as well.
		 */
		if (!strcmp(node->name, "multifunc-device")) {
			for_each_child_of_node(node, node2) {
				if (__of_pci_pci_compare(node2, devfn)) {
					of_node_put(node);
					return node2;
				}
			}
		}
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(of_pci_find_child_device);

/**
 * of_pci_parse_bus_range() - parse the bus-range property of a PCI device
 * @node: device node
 * @res: address to a struct resource to return the bus-range
 *
 * Returns 0 on success or a negative error-code on failure.
 */
int of_pci_parse_bus_range(struct device_node *node, struct resource *res)
{
	const __be32 *values;
	int len;

	values = of_get_property(node, "bus-range", &len);
	if (!values || len < sizeof(*values) * 2)
		return -EINVAL;

	res->name = node->name;
	res->start = be32_to_cpup(values++);
	res->end = be32_to_cpup(values);
	res->flags = IORESOURCE_BUS;

	return 0;
}
EXPORT_SYMBOL_GPL(of_pci_parse_bus_range);

static atomic_t of_domain_nr = ATOMIC_INIT(-1);

/*
 * Get the maximum value for a domain number from the device tree
 */
static int of_get_max_pci_domain_nr(void)
{
	struct alias_prop *app;
	int max_domain = -1;

	mutex_lock(&of_aliases_mutex);
	list_for_each_entry(app, &aliases_lookup, link) {
		if (strncmp(app->stem, "pci-domain", 10) != 0)
			continue;

		max_domain = max(max_domain, app->id);
	}
	mutex_unlock(&of_aliases_mutex);

	return max_domain;
}

/**
 * This function will try to obtain the host bridge domain number by
 * using of_alias_get_id() call with "pci-domain" as a stem. If that
 * fails, a local allocator will be used. The local allocator can
 * be requested to return a new domain_nr if the information is missing
 * from the device tree.
 *
 * @node: device tree node with the domain information
 * @allocate_if_missing: if DT lacks information about the domain nr,
 * allocate a new number.
 *
 * Returns the associated domain number from DT, or a new domain number
 * if DT information is missing and @allocate_if_missing is true. If
 * @allocate_if_missing is false then the last allocated domain number
 * will be returned.
 */
int of_pci_get_domain_nr(struct device_node *node, bool allocate_if_missing)
{
	int domain;

	domain = atomic_read(&of_domain_nr);
	if (domain == -1) {
		/* first run, get max defined domain nr in device tree */
		domain = of_get_max_pci_domain_nr();
		/* then set the start value for allocator to be max + 1 */
		atomic_set(&of_domain_nr, domain + 1);
	}
	domain = of_alias_get_id(node, "pci-domain");
	if (domain == -ENODEV) {
		domain = atomic_read(&of_domain_nr);
		if (allocate_if_missing)
			atomic_inc(&of_domain_nr);
	}

	return domain;
}
EXPORT_SYMBOL_GPL(of_pci_get_domain_nr);
