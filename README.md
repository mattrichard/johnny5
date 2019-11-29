# johnny5
ROS packages for the Lynxmotion Johnny 5 Robot

### Bluetooth device

```sh
hcitool scan
sudo rfcomm bind 0 98:D3:33:10:00:3A 1
sudo chmod a+rw /dev/rfcomm0
```
