int16 FIX_TYPE_NONE = 0
int16 FIX_TYPE_RAW = 1
int16 FIX_TYPE_FLOAT = 2
int16 FIX_TYPE_FIXED = 3
int16 FIX_TYPE_FIXED_SBAS = 4

Header header
float64 utc_time # In seconds
string utm_zone
float64 easting # In meters
float64 northing # In meters
int16 fix_type # One of FIX_TYPE_*
int16 num_satellites # Between 3 and 14
float64 hdop 
float64 antenna_height
float64 geoidal_separation
int32 differential_corrections_age
string differential_station_id
float64[9] position_covariance
