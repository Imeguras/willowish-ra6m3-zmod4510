# willowish-ra6m5-zmod4510
The repo's name is wrong! this is for the ra6m5 not ra6m3,

## Important Note before usage
you must insert the .PEM certificate as a header inside the project check server_certificate_EXAMPLE.h as a guide
**Pro tip:** dont forget to remove the EXAMPLE from the name, obviously...

## Requirements to run
To run this project in a development environment as its currently configured you will need:
Software:
- [e2studio](https://www.renesas.com/us/en/products/software-tools/tools/ide/e2studio.html)
- [FSPv5.1.0](https://github.com/renesas/fsp/releases/tag/v5.1.0)(Although some version of the FSP will be installed tipically when you install e2studio, they wont stop updating it, you should be able to run some other modern fsps, but this one was the one that it worked for me, its noted that zmod has some problems in this version which is kinda funny because projects with other fsps didnt "work", not their fault though i think)
- [Segger J-Link](https://www.segger.com/downloads/jlink/#J-LinkSoftwareAndDocumentationPack)(Dont mind this one i **think** it comes with the IDE, if you're on windows too bad, i don't know if its included in the installer)
- [RA family compiler]()(I think you install this when you're downloading the IDE)
- (Optional) [Git]()(this comes in handy specially because there wasnt proper care put into versioning the project)

Hardware:
- [Renesas RA6M5 Evaluation kit]()(This is the board i used, it should work with the other RA family boards, changing the board in config.xml has proven to be unstable for me in the past)
- [Zmod4510]()
- [HS]
