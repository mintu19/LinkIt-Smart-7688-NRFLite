# LinkIt-Smart-7688-NRFLite
Python Mraa Demo implementation to connect LinkIt Smart 7688 to nrf24l01+ module.
This is python port of NRFLite Arduino Library. [NRFLite](https://github.com/dparson55/NRFLite)

### Tested
Register Read and Write Operations
Sending Data to another module connected to arduino

### To-Do
Test Receive mode
Test other functions

## LinkIt SPI Bus
LinkIt has some silicone level problem with SPI bus (?) due to which we will be using Software SPI. Check SPI.py file for pin details.

### Monotonic
`pip install monotonic`