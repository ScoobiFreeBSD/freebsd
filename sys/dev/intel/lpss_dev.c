#include <sys/param.h>
#include <sys/module.h>
#include <sys/kernel.h>
#include <sys/systm.h>

#include <sys/conf.h>
#include <sys/uio.h>
#include <sys/bus.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

struct lpss_softc {
	device_t	sc_dev;
};

static const struct {
	uint16_t vendor;
	uint16_t device;
} lpss_pci_ids[] = {
	{ 0x8086, 0x9d61 },
	{ 0, 0 }
};

static int
lpss_pci_probe(device_t dev)
{
	int i;

	for (i = 0; lpss_pci_ids[i].vendor != 0; ++i) {
		if (pci_get_vendor(dev) == lpss_pci_ids[i].vendor &&
		    pci_get_device(dev) == lpss_pci_ids[i].device) {
			device_set_desc(dev, "Intel LPSS PCI Driver");
			return (BUS_PROBE_DEFAULT);
		}
	}
	return ENXIO;
}

static int
lpss_pci_attach(device_t dev)
{
	struct lpss_softc *sc;

	sc = device_get_softc(dev);

	return (0);
}

static int
lpss_pci_detach(device_t dev)
{
	return (0);
}

static int
lpss_pci_shutdown(device_t dev)
{
	return (0);
}

static int
lpss_pci_suspend(device_t dev)
{
	return (0);
}

static int
lpss_pci_resume(device_t dev)
{
	return (0);
}

static device_method_t lpss_pci_methods[] = {
    /* Device interface */
    DEVMETHOD(device_probe,		lpss_pci_probe),
    DEVMETHOD(device_attach,		lpss_pci_attach),
    DEVMETHOD(device_detach,		lpss_pci_detach),
    DEVMETHOD(device_shutdown,		lpss_pci_shutdown),
    DEVMETHOD(device_suspend,		lpss_pci_suspend),
    DEVMETHOD(device_resume,		lpss_pci_resume),

#if 0
    DEVMETHOD(bus_alloc_resource,	lpss_bus_alloc_resource),
    DEVMETHOD(bus_release_resource,	lpss_bus_release_resource),
    DEVMETHOD(bus_get_resource,		lpss_bus_get_resource),
    DEVMETHOD(bus_read_ivar,		lpss_bus_read_ivar),
    DEVMETHOD(bus_setup_intr,		lpss_bus_setup_intr),
    DEVMETHOD(bus_teardown_intr,	lpss_bus_teardown_intr),
    DEVMETHOD(bus_print_child,		lpss_bus_print_child),
    DEVMETHOD(bus_child_pnpinfo_str,	lpss_bus_child_pnpinfo_str),
    DEVMETHOD(bus_child_location_str,	lpss_bus_child_location_str),
#endif

    DEVMETHOD_END
};

static driver_t lpss_pci_driver = {
	"lpss",
	lpss_pci_methods,
	sizeof(struct lpss_softc),
};

static devclass_t lpss_devclass;

DRIVER_MODULE(lpss, pci, lpss_pci_driver, lpss_devclass, 0, 0);
