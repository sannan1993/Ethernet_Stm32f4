
#ifndef ADE7758_H
#define ADE7758_H



#define  RESERVED    	 0x00

#define  AWATTHR    	 0x01
#define  BWATTHR     	 0x02
#define  CWATTHR     	 0x03

#define  AVARHR      	 0x04
#define  BVARHR      	 0x05
#define  CVARHR      	 0x06

#define  AVAHR       	 0x07
#define  BVAHR       	 0x08
#define  CVAHR       	 0x09

#define  AIRMS      	 0x0A
#define  BIRMS      	 0x0B
#define  CIRMS      	 0x0C

#define  AVRMS      	 0x0D
#define  BVRMS      	 0x0E
#define  CVRMS         0x0F

#define  FREQUECNY   	 0x10
#define	 TEMP 			   0x11

#define	 WFORM 		 	   0x12
#define	 OPMODE 		   0x13
#define	 MMODE			   0x14
#define	 WAVMODE	     0x15

#define  COMPMODE		   0x16
#define  LCYCMODE 		 0x17
#define  MASK 			   0x18

#define  STATUS 		   0x19
#define  RSTATUS		   0x1A
#define  ZXTOUT        0x1B

#define  LINECYC       0x1C
#define  SAGCYC        0x1D 
#define  SAGLVL        0x1E

#define  VPINTLVL 		 0x1F
#define  IPINTLVL		   0x20 
#define  VPEAK 			   0x21

#define  IPEAK 			   0x22
#define  GAIN 			   0x23

#define  VRMSGAIN      0x24
#define  BVRMSGAIN     0x25
#define  CVRMSGAIN     0x26 

#define  AIGAIN 		   0x27 
#define  BIGAIN 		   0x28 
#define  CIGAIN 		   0x29 

#define	 AWG           0x2A
#define	 BWG			     0x2B
#define	 CWG			     0x2C

#define	AVARG 			   0x2D
#define	BVARG 			   0x2E
#define	CVARG 			   0x2F

#define	AVAG 			     0x30
#define	BVAG 			     0x31
#define	CVAG 			     0x32

#define	AVRMSOS			   0x33
#define	BVRMSOS 		   0x34
#define	CVRMSOS				 0x35

#define	AIRMSOS			   0x36
#define	BIRMSOS 		   0x37
#define	CIRMSOS			   0x38

#define	AWATTOS 		   0x39
#define	BWATTOS 		   0x3A
#define	CWATTOS 		   0x3B

#define	AVAROS				 0x3C
#define	BVAROS				 0x3D
#define	CVAROS 				 0x3E

#define	APHCAL 			   0x3F 
#define	BPHCAL 		     0x40 
#define	CPHCAL 	  	   0x41 

#define	WDIV 					 0x42 
#define	VARDIV 				 0x43
#define	VADIV					 0x44

#define	APCFNUM 			 0x45 
#define	APCFDEN 	     0x46 
#define VARCFNUM 	     0x47 
#define	VARCFDEN	     0x48 
// 0x49 to 0x7D
#define	CHKSUM 		     0x7E
#define	VERSION		     0x7F



#endif
