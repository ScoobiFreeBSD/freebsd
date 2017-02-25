#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>

#include <sys/kernel.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>

struct iwl_pci_softc
{
};

static int
iwl_pci_probe(device_t dev)
{
	return -1;
}

static int
iwl_pci_attach(device_t dev)
{
	return -1;
}

static int
iwl_pci_detach(device_t dev)
{
	return -1;
}


static int
iwl_pci_shutdown(device_t dev)
{
	return -1;
}

static int
iwl_pci_suspend(device_t dev)
{
	return -1;
}

static int
iwl_pci_resume(device_t dev)
{
	return -1;
}


static device_method_t iwl_pci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		iwl_pci_probe),
	DEVMETHOD(device_attach,	iwl_pci_attach),
	DEVMETHOD(device_detach,	iwl_pci_detach),
	DEVMETHOD(device_shutdown,	iwl_pci_shutdown),
	DEVMETHOD(device_suspend,	iwl_pci_suspend),
	DEVMETHOD(device_resume,	iwl_pci_resume),

	{ 0,0 }
};
static driver_t iwl_pci_driver = {
	"iwl",
	iwl_pci_methods,
	sizeof (struct iwl_pci_softc)
};
static	devclass_t iwl_devclass;
DRIVER_MODULE(iwl_pci, pci, iwl_pci_driver, iwl_devclass, 0, 0);
MODULE_VERSION(iwl_pci, 1);
MODULE_DEPEND(iwl_pci, wlan, 1, 1, 1);		/* 802.11 media layer */

