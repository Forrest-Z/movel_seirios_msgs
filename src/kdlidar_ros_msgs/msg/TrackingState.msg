# Tracking state from kdlidar

# System is currently initialising / in need of initialisation
uint8 INITIALISING = 1
# Tracking is happening
uint8 TRACKING = 2
# Tracking is lost - relocalisation required
uint8 LOST = 3


Header header
uint8 tracking_state
string msg