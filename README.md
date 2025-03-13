
# Unbind the device from its current driver
echo "0000:b3:00.0" > /sys/bus/pci/drivers/octeon_ep/unbind

# Bind the device to the vfio-pci driver
echo "vfio-pci" > /sys/bus/pci/devices/0000:b3:00.0/driver_override
echo "0000:b3:00.0" > /sys/bus/pci/drivers/vfio-pci/bind

# Get the iommu group id after vfio binding
readlink /sys/bus/pci/devices/0000\:b3\:00.0/iommu_group | awk -F '/' '{print $NF}'

# Get memory regions in PCI
lspci -s 0000:b3:00.0 -vvv
