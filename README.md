# IHATECATS
Infrared motion-sensing anti-cat platform for chasing away feral cats

# Introduction
I don't actually hate cats, but when they pry their way through the skirting to get under my house and dig into the insulation under my bedroom, it's stinky (on account of the fact that CATS ARE STUPID AND PISS ON EVERYTHING) and expensive to repair (because let's face it, who wants to deal with fiberglass insulation SOAKED IN CAT PISS). I'd repair the skirting (and pay some fool a ton of money to replace the insulation), but that would trap any cats currently under my house, and in addition to being cruel and gruesome, it would eventually stink even worse (at least for a while).

This project started out as a silly rant about how I wanted one of those automated machine gun turrets from Aliens to get rid of the cats under my house. I allowed that I'd be willing to settle for an automated Airsoft SMG, on account of actual machine guns being so hard to come by (legally), but even that would do more harm to the cats than I really want (because it's not really the cats' fault that they're STUPID AND PISS ON EVERYTHING), and it's probably illegal, too. Finally, I settled on a less harmful solution: compressed air.

# Hardware
This project is based on Adafruit's Broadcomm WICED WiFi Feather. It uses a simple motion detector to detect the cats' movement, then sprays compressed air at them, because initial testing indicated quite clearly that cats hate the fuck out of being sprayed with compressed air. The system also includes an infrared camera with TTL serial output to capture each activation, because initial testing also revealed that spraying cats with compressed air is FUCKING HILARIOUS. The images are temporarily stored on an SD card, and the system utilizes the WICED Feather's IOT capabilities to deliver the images to the Internet, because looking at pictures of spazzed-out cats is more enjoyable if you don't have to go into a dank crawlspace to get them.

The actual components used are as follows:
* Adafruit Broadcomm WICED WiFi Feather https://www.adafruit.com/products/3056
* Adafruit Adalogger RTC/SD FeatherWing https://www.adafruit.com/products/2922
* Adafruit OLED FeatherWing https://www.adafruit.com/products/2900
* TTL Serial Infrared Camera https://www.adafruit.com/products/613
* Infrared Motion Sensor https://www.sparkfun.com/products/13285
