/*
 * Registers by Arducam https://github.com/ArduCAM/Arduino/blob/master/ArduCAM/ov5640_regs.h
 * https://github.com/ArduCAM/Arduino/blob/master/ArduCAM/ArduCAM.cpp
 */

#include "ch.h"
#include "hal.h"
#include "ov5640.h"
#include "pi2c.h"
#include "board.h"
#include "debug.h"
#include "padc.h"
#include "si446x.h"
#include <string.h>
#include "pktradio.h"

static uint32_t lightIntensity;
static uint8_t error;

struct regval_list {
	uint16_t reg;
	uint8_t val;
};

static const struct regval_list OV5640YUV_Sensor_Dvp_Init[] =
{
	{ 0x4740, 0x24 },
		
		{ 0x4050, 0x6e },
    { 0x4051, 0x8f },
		
		{ 0x3008, 0x42 }, 
		{ 0x3103, 0x03 }, 
		{ 0x3017, 0x7f }, 
		{ 0x3018, 0xff }, 		
		{ 0x302c, 0x02 }, 		
		{ 0x3108, 0x31 }, 	
		{ 0x3630, 0x2e },//2e
		{ 0x3632, 0xe2 }, 
		{ 0x3633, 0x23 },//23 
		{ 0x3621, 0xe0 }, 
		{ 0x3704, 0xa0 }, 
		{ 0x3703, 0x5a }, 
		{ 0x3715, 0x78 }, 
		{ 0x3717, 0x01 }, 
		{ 0x370b, 0x60 }, 
		{ 0x3705, 0x1a }, 
		{ 0x3905, 0x02 }, 
		{ 0x3906, 0x10 }, 
		{ 0x3901, 0x0a }, 
		{ 0x3731, 0x12 }, 
		{ 0x3600, 0x08 }, 
		{ 0x3601, 0x33 }, 
		{ 0x302d, 0x60 }, 
		{ 0x3620, 0x52 }, 
		{ 0x371b, 0x20 }, 
		{ 0x471c, 0x50 }, 
		
		{ 0x3a18, 0x00 }, 
		{ 0x3a19, 0xf8 }, 
		
		{ 0x3635, 0x1c },//1c
		{ 0x3634, 0x40 }, 
		{ 0x3622, 0x01 }, 
 
		{ 0x3c04, 0x28 }, 
		{ 0x3c05, 0x98 }, 
		{ 0x3c06, 0x00 }, 
		{ 0x3c07, 0x08 }, 
		{ 0x3c08, 0x00 }, 
		{ 0x3c09, 0x1c }, 
		{ 0x3c0a, 0x9c }, 
		{ 0x3c0b, 0x40 },  		

		{ 0x3820, 0x47 }, 
		{ 0x3821, 0x00 }, //07

		//windows setup
		{ 0x3800, 0x00 }, 
		{ 0x3801, 0x00 }, 
		{ 0x3802, 0x00 }, 
		{ 0x3803, 0x04 }, 
		{ 0x3804, 0x0a }, 
		{ 0x3805, 0x3f }, 
		{ 0x3806, 0x07 }, 
		{ 0x3807, 0x9b }, 
		{ 0x3808, 0x05 },  
		{ 0x3809, 0x00 }, 
		{ 0x380a, 0x03 }, 
		{ 0x380b, 0xc0 }, 		
		{ 0x3810, 0x00 }, 
		{ 0x3811, 0x10 }, 
		{ 0x3812, 0x00 }, 
		{ 0x3813, 0x06 }, 
		{ 0x3814, 0x31 }, 
		{ 0x3815, 0x31 },		
		
		{ 0x3034, 0x1a }, 
		{ 0x3035, 0x01 }, //15fps
		{ 0x3036, 0x46 }, 
		{ 0x3037, 0x03 }, 
		{ 0x3038, 0x00 }, 
		{ 0x3039, 0x00 }, 
		
		{ 0x380c, 0x07 }, 
		{ 0x380d, 0x68 }, 
		{ 0x380e, 0x03 }, //03
		{ 0x380f, 0xd8 }, //d8
		 
		{ 0x3c01, 0xb4 }, 
		{ 0x3c00, 0x04 }, 
		{ 0x3a08, 0x00 }, 
		{ 0x3a09, 0x93 }, 
		{ 0x3a0e, 0x06 },
		{ 0x3a0a, 0x00 }, 
		{ 0x3a0b, 0x7b }, 		 
		{ 0x3a0d, 0x08 }, 
		
		{ 0x3a00, 0x3c }, //15fps-10fps
		{ 0x3a02, 0x05 }, 
		{ 0x3a03, 0xc4 }, 
		{ 0x3a14, 0x05 }, 
		{ 0x3a15, 0xc4 }, 
		
		{ 0x3618, 0x00 }, 
		{ 0x3612, 0x29 }, 
		{ 0x3708, 0x64 }, 
		{ 0x3709, 0x52 }, 
		{ 0x370c, 0x03 },
		
		{ 0x4001, 0x02 }, 
		{ 0x4004, 0x02 }, 
		{ 0x3000, 0x00 }, 
		{ 0x3002, 0x1c }, 
		{ 0x3004, 0xff }, 
		{ 0x3006, 0xc3 }, 
		{ 0x300e, 0x58 }, 
		{ 0x302e, 0x00 }, 
		{ 0x4300, 0x30 }, 
		{ 0x501f, 0x00 }, 
		{ 0x4713, 0x03 }, // Compression mode
		{ 0x4404, 0x34 }, // Gated Clock Enabled (default 0x24)
		{ 0x4407, 0x04 }, 
		{ 0x460b, 0x35 }, 
		{ 0x460c, 0x22 },//add by bright 
	  { 0x3824, 0x01 },//add by bright 
		{ 0x5001, 0xa3 }, 		
		
		{ 0x3406, 0x01 },//awbinit
		{ 0x3400, 0x06 },
		{ 0x3401, 0x80 },
		{ 0x3402, 0x04 },
		{ 0x3403, 0x00 },
		{ 0x3404, 0x06 },
		{ 0x3405, 0x00 },
	  //awb           
		{ 0x5180, 0xff }, 
		{ 0x5181, 0xf2 },    
		{ 0x5182, 0x00 },   
		{ 0x5183, 0x14 },    
		{ 0x5184, 0x25 },    
		{ 0x5185, 0x24 },    
		{ 0x5186, 0x16 },    
		{ 0x5187, 0x16 },    
		{ 0x5188, 0x16 },    
		{ 0x5189, 0x62 },    
		{ 0x518a, 0x62 },    
		{ 0x518b, 0xf0 },    
		{ 0x518c, 0xb2 },    
		{ 0x518d, 0x50 },    
		{ 0x518e, 0x30 },    
		{ 0x518f, 0x30 },    
		{ 0x5190, 0x50 },    
		{ 0x5191, 0xf8 },    
		{ 0x5192, 0x04 },   
		{ 0x5193, 0x70 },    
		{ 0x5194, 0xf0 },    
		{ 0x5195, 0xf0 },    
		{ 0x5196, 0x03 },   
		{ 0x5197, 0x01 },   
		{ 0x5198, 0x04 },   
		{ 0x5199, 0x12 },    
		{ 0x519a, 0x04 },   
		{ 0x519b, 0x00 },   
		{ 0x519c, 0x06 },   
		{ 0x519d, 0x82 },    
		{ 0x519e, 0x38 },  
		//color matrix  	                                        	                                  
		{ 0x5381, 0x1e },  
		{ 0x5382, 0x5b }, 
		{ 0x5383, 0x14 }, 
		{ 0x5384, 0x06 }, 
		{ 0x5385, 0x82 }, 
		{ 0x5386, 0x88 }, 
		{ 0x5387, 0x7c }, 
		{ 0x5388, 0x60 }, 
		{ 0x5389, 0x1c }, 
		{ 0x538a, 0x01 }, 
		{ 0x538b, 0x98 }, 
		//sharp&noise
		{ 0x5300, 0x08 }, 
		{ 0x5301, 0x30 }, 
		{ 0x5302, 0x3f }, 
		{ 0x5303, 0x10 },
		{ 0x5304, 0x08 }, 
		{ 0x5305, 0x30 }, 
		{ 0x5306, 0x18 }, 
		{ 0x5307, 0x28 },
		{ 0x5309, 0x08 }, 
		{ 0x530a, 0x30 }, 
		{ 0x530b, 0x04 }, 
		{ 0x530c, 0x06 }, 	 
		//gamma                         
		{ 0x5480, 0x01 },
		{ 0x5481, 0x06 }, 
		{ 0x5482, 0x12 },  
		{ 0x5483, 0x24 },  
		{ 0x5484, 0x4a }, 
		{ 0x5485, 0x58 },  
		{ 0x5486, 0x65 },  
		{ 0x5487, 0x72 },  
		{ 0x5488, 0x7d },  
		{ 0x5489, 0x88 },  
		{ 0x548a, 0x92 },  
		{ 0x548b, 0xa3 },  
		{ 0x548c, 0xb2 },  
		{ 0x548d, 0xc8 },  
		{ 0x548e, 0xdd },  
		{ 0x548f, 0xf0 }, 
		{ 0x5490, 0x15 }, 	  
		//UV adjust                              	                    
		{ 0x5580, 0x06 }, 		
		{ 0x5583, 0x40 }, 
		{ 0x5584, 0x20 }, 
		{ 0x5589, 0x10 }, 
		{ 0x558a, 0x00 }, 
		{ 0x558b, 0xf8 },                                 
		//lens shading                                      
		{ 0x5000, 0xa7 },                                      	
		{ 0x5800, 0x20 }, 
		{ 0x5801, 0x19 }, 
		{ 0x5802, 0x17 }, 
		{ 0x5803, 0x16 }, 
		{ 0x5804, 0x18 }, 
		{ 0x5805, 0x21 }, 
		{ 0x5806, 0x0F }, 
		{ 0x5807, 0x0A }, 
		{ 0x5808, 0x07 }, 
		{ 0x5809, 0x07 }, 
		{ 0x580a, 0x0A }, 
		{ 0x580b, 0x0C }, 
		{ 0x580c, 0x0A }, 
		{ 0x580d, 0x03 }, 
		{ 0x580e, 0x01 }, 
		{ 0x580f, 0x01 }, 
		{ 0x5810, 0x03 }, 
		{ 0x5811, 0x09 }, 
		{ 0x5812, 0x0A }, 
		{ 0x5813, 0x03 }, 
		{ 0x5814, 0x01 }, 
		{ 0x5815, 0x01 }, 
		{ 0x5816, 0x03 }, 
		{ 0x5817, 0x08 }, 
		{ 0x5818, 0x10 }, 
		{ 0x5819, 0x0A }, 
		{ 0x581a, 0x06 }, 
		{ 0x581b, 0x06 }, 
		{ 0x581c, 0x08 }, 
		{ 0x581d, 0x0E }, 
		{ 0x581e, 0x22 }, 
		{ 0x581f, 0x18 }, 
		{ 0x5820, 0x13 }, 
		{ 0x5821, 0x12 }, 
		{ 0x5822, 0x16 }, 
		{ 0x5823, 0x1E }, 
		{ 0x5824, 0x64 }, 
		{ 0x5825, 0x2A }, 
		{ 0x5826, 0x2C }, 
		{ 0x5827, 0x2A }, 
		{ 0x5828, 0x46 }, 
		{ 0x5829, 0x2A }, 
		{ 0x582a, 0x26 }, 
		{ 0x582b, 0x24 }, 
		{ 0x582c, 0x26 }, 
		{ 0x582d, 0x2A }, 
		{ 0x582e, 0x28 }, 
		{ 0x582f, 0x42 }, 
		{ 0x5830, 0x40 }, 
		{ 0x5831, 0x42 }, 
		{ 0x5832, 0x08 }, 
		{ 0x5833, 0x28 }, 
		{ 0x5834, 0x26 }, 
		{ 0x5835, 0x24 }, 
		{ 0x5836, 0x26 }, 
		{ 0x5837, 0x2A }, 
		{ 0x5838, 0x44 }, 
		{ 0x5839, 0x4A }, 
		{ 0x583a, 0x2C }, 
		{ 0x583b, 0x2a }, 
		{ 0x583c, 0x46 }, 
		{ 0x583d, 0xCE }, 	
		
		{ 0x5688, 0x22 }, 
		{ 0x5689, 0x22 }, 
		{ 0x568a, 0x42 }, 
		{ 0x568b, 0x24 }, 
		{ 0x568c, 0x42 }, 
		{ 0x568d, 0x24 }, 
		{ 0x568e, 0x22 }, 
		{ 0x568f, 0x22 }, 
		
		{ 0x5025, 0x00 }, 
		
		{ 0x3a0f, 0x30 },
		{ 0x3a10, 0x28 }, 
		{ 0x3a1b, 0x30 }, 
		{ 0x3a1e, 0x28 }, 
		{ 0x3a11, 0x61 }, 
		{ 0x3a1f, 0x10 }, 
		
		{ 0x4005, 0x1a },
		{ 0x3406, 0x00 },//awbinit
    { 0x3503, 0x00 },//awbinit
		{ 0x3008, 0x02 }, 
{ 0xffff, 0xff }, 
};



//2592x1944 QSXGA
static const struct regval_list OV5640_JPEG_QSXGA[]  =
{
	{0x3820 ,0x47}, 
		{0x3821 ,0x20}, 
		{0x3814 ,0x11}, 
		{0x3815 ,0x11}, 
		{0x3803 ,0x00}, 
		{0x3807 ,0x9f}, 
		{0x3808 ,0x0a}, 
		{0x3809 ,0x20}, 
		{0x380a ,0x07}, 
		{0x380b ,0x98},
		{0x380c ,0x0b},                                                                    
		{0x380d ,0x1c}, 
		{0x380e ,0x07},                                                         
		{0x380f ,0xb0},                                                          
		{0x3813 ,0x04},                                                     
		{0x3618 ,0x04},                        
		{0x3612 ,0x4b},                                               
		{0x3708 ,0x64},                
		{0x3709 ,0x12},                                    
		{0x370c ,0x00},  
		{0x3a02 ,0x07},                             
		{0x3a03 ,0xb0},                        
		{0x3a0e ,0x06},                    
		{0x3a0d ,0x08}, 
		{0x3a14 ,0x07}, 
		{0x3a15 ,0xb0}, 
		{0x4001 ,0x02}, 
		{0x4004 ,0x06}, 
		{0x3002 ,0x00}, 
		{0x3006 ,0xff}, 
		{0x3824 ,0x04}, 
		{0x5001 ,0x83}, 
		{0x3036 ,0x69}, 
		{0x3035 ,0x31}, 
		{0x4005 ,0x1A},
{0xffff, 0xff}, 
};

//5MP
static const struct regval_list OV5640_5MP_JPEG[] __attribute__((unused)) =
{
	{0x3800 ,0x00},                            
	{0x3801 ,0x00},                                                            
	{0x3802 ,0x00},                            
	{0x3803 ,0x00},   
	{0x3804 ,0xA },           
	{0x3805 ,0x3f},          
	{0x3806 ,0x7 },          
	{0x3807 ,0x9f},          
	{0x3808 ,0xA },                               
	{0x3809 ,0x20},  
	{0x380a ,0x7 },           
	{0x380b ,0x98}, 
	{0x380c ,0xc },                        
	{0x380d ,0x80},                      
	{0x380e ,0x7 }, 
	{0x380f ,0xd0}, 
	{0x5001 ,0xa3}, 
	{0x5680 ,0x0 }, 
	{0x5681 ,0x0 }, 
	{0x5682 ,0xA }, 
	{0x5683 ,0x20}, 
	{0x5684 ,0x0 }, 
	{0x5685 ,0x0 }, 
	{0x5686 ,0x7 }, 
	{0x5687 ,0x98}, 	
	{0xffff, 0xff},	
};

//320x240 QVGA
static const struct regval_list OV5640_QSXGA2QVGA[]  =
{
	{0x3800 ,0x00},
	{0x3801 ,0x00},
	{0x3802 ,0x00},
	{0x3803 ,0x00},
	{0x3804 ,0xA },
	{0x3805 ,0x3f},
	{0x3806 ,0x7 },
	{0x3807 ,0x9f},
	{0x3808 ,0x1 },
	{0x3809 ,0x40},
	{0x380a ,0x0 },
	{0x380b ,0xf0},
	{0x380c ,0xc },
	{0x380d ,0x80},
	{0x380e ,0x7 },
	{0x380f ,0xd0},
	{0x5001 ,0xa3},
	{0x5680 ,0x0 },
	{0x5681 ,0x0 },
	{0x5682 ,0xA },
	{0x5683 ,0x20},
	{0x5684 ,0x0 },
	{0x5685 ,0x0 },
	{0x5686 ,0x7 },
	{0x5687 ,0x98},
	{0xffff, 0xff},
};

//320x240 QQVGA
static const struct regval_list OV5640_QSXGA2QQVGA[]  =
{
	{0x3800 ,0x00},
	{0x3801 ,0x00},
	{0x3802 ,0x00},
	{0x3803 ,0x00},
	{0x3804 ,0xA },
	{0x3805 ,0x3f},
	{0x3806 ,0x7 },
	{0x3807 ,0x9f},
	{0x3808 ,0x0 },
	{0x3809 ,0xA0},
	{0x380a ,0x0 },
	{0x380b ,0x70},
	{0x380c ,0xc },
	{0x380d ,0x80},
	{0x380e ,0x7 },
	{0x380f ,0xd0},
	{0x5001 ,0xa3},
	{0x5680 ,0x0 },
	{0x5681 ,0x0 },
	{0x5682 ,0xA },
	{0x5683 ,0x20},
	{0x5684 ,0x0 },
	{0x5685 ,0x0 },
	{0x5686 ,0x7 },
	{0x5687 ,0x98},
	{0xffff, 0xff},
};

//640x480 VGA
static const struct regval_list OV5640_QSXGA2VGA[]  =
{
		{0x3800 ,0x00}, 
		{0x3801 ,0x00}, 
		{0x3802 ,0x00}, 
		{0x3803 ,0x00},                                                                                                            
		{0x3804 ,0xA },                                  
		{0x3805 ,0x3f},                                           
		{0x3806 ,0x7 },                                     
		{0x3807 ,0x9f},                                     
		{0x3808 ,0x2 },                                                                                          
		{0x3809 ,0x80},                               
		{0x380a ,0x1 },                               
		{0x380b ,0xe0},                               
		{0x380c ,0xc },                  
		{0x380d ,0x80},                                                            
		{0x380e ,0x7 },               
		{0x380f ,0xd0},       
		{0x5001 ,0xa3},        
		{0x5680 ,0x0 },                                  
		{0x5681 ,0x0 },       
		{0x5682 ,0xA },   
		{0x5683 ,0x20},
		{0x5684 ,0x0 },
		{0x5685 ,0x0 }, 
		{0x5686 ,0x7 }, 
		{0x5687 ,0x98}, 	
{0xffff, 0xff}, 
};

//800x480 WVGA
static const struct regval_list OV5640_QSXGA2WVGA[] __attribute__((unused)) =
{
	{0x3800 ,0x00}, 
	{0x3801 ,0x00}, 
	{0x3802 ,0x00}, 
	{0x3803 ,0x00},                                                                                                            
	{0x3804 ,0xA },                                  
	{0x3805 ,0x3f},                                           
	{0x3806 ,0x7 },                                     
	{0x3807 ,0x9f},
	{0x3808 ,0x3 },
	{0x3809 ,0x20},
	{0x380a ,0x1 },                               
	{0x380b ,0xe0},                               
	{0x380c ,0xc },                  
	{0x380d ,0x80},
	{0x380e ,0x7 },               
	{0x380f ,0xd0},   
	{0x3810, 0x00},
	{0x3811, 0x10},
	{0x3812, 0x01},
	{0x3813, 0x48},    
	{0x5001 ,0xa3},        
	{0x5680 ,0x0 },                                  
	{0x5681 ,0x0 },       
	{0x5682 ,0xA },   
	{0x5683 ,0x20},
	{0x5684 ,0x0 },
	{0x5685 ,0x0 }, 
	{0x5686 ,0x7 }, 
	{0x5687 ,0x98}, 	
	{0xffff, 0xff},	
};

//352x288 CIF
static const struct regval_list OV5640_QSXGA2CIF[] __attribute__((unused)) =
{
	{0x3800 ,0x00}, 
	{0x3801 ,0x00}, 
	{0x3802 ,0x00}, 
	{0x3803 ,0x00},                                                                                                            
	{0x3804 ,0xA },                                  
	{0x3805 ,0x3f},                                           
	{0x3806 ,0x7 },                                     
	{0x3807 ,0x9f},                                     
	{0x3808 ,0x1 },                                                                                          
	{0x3809 ,0x60},                               
	{0x380a ,0x1 },                               
	{0x380b ,0x20},                               
	{0x380c ,0xc },                  
	{0x380d ,0x80},                                                            
	{0x380e ,0x7 },               
	{0x380f ,0xd0},   
	{0x3810, 0x00},
	{0x3811, 0x10},
	{0x3812, 0x00},
	{0x3813, 0x70},    
	{0x5001 ,0xa3},        
	{0x5680 ,0x0 },                                  
	{0x5681 ,0x0 },       
	{0x5682 ,0xA },   
	{0x5683 ,0x20},
	{0x5684 ,0x0 },
	{0x5685 ,0x0 }, 
	{0x5686 ,0x7 }, 
	{0x5687 ,0x98}, 	
	{0xffff, 0xff},	
};

//1280x960 SXGA
static const struct regval_list OV5640_QSXGA2SXGA[] __attribute__((unused)) =
{
	{0x3800 ,0x00},
	{0x3801 ,0x00},
	{0x3802 ,0x00},
	{0x3803 ,0x00},
	{0x3804 ,0xA },
	{0x3805 ,0x3f},
	{0x3806 ,0x7 },
	{0x3807 ,0x9f},
	{0x3808 ,0x5 },
	{0x3809 ,0x0 },
	{0x380a ,0x3 },
	{0x380b ,0xc0},
	{0x380c ,0xc },
	{0x380d ,0x80},
	{0x380e ,0x7 },
	{0x380f ,0xd0},
	{0x5001 ,0xa3},
	{0x5680 ,0x0 },
	{0x5681 ,0x0 },
	{0x5682 ,0xA },
	{0x5683 ,0x20},
	{0x5684 ,0x0 },
	{0x5685 ,0x0 },
	{0x5686 ,0x7 },
	{0x5687 ,0x98},
	{0xffff, 0xff},
};

//2048x1536 QXGA
static const struct regval_list OV5640_QSXGA2QXGA[] __attribute__((unused)) =
{
	{0x3800 ,0x00},
	{0x3801 ,0x00},
	{0x3802 ,0x00},
	{0x3803 ,0x00},
	{0x3804 ,0xA },
	{0x3805 ,0x3f},
	{0x3806 ,0x7 },
	{0x3807 ,0x9f},
	{0x3808 ,0x8 },
	{0x3809 ,0x0 },
	{0x380a ,0x6 },
	{0x380b ,0x0 },
	{0x380c ,0xc },
	{0x380d ,0x80},
	{0x380e ,0x7 },
	{0x380f ,0xd0},
	{0x5001 ,0xa3},
	{0x5680 ,0x0 },
	{0x5681 ,0x0 },
	{0x5682 ,0xA },
	{0x5683 ,0x20},
	{0x5684 ,0x0 },
	{0x5685 ,0x0 },
	{0x5686 ,0x7 },
	{0x5687 ,0x98},	
	{0xffff, 0xff},
};


//1600x1200 UXGA
static const struct regval_list OV5640_QSXGA2UXGA[]  =	
{
	{0x3800 ,0x00},
	{0x3801 ,0x00},
	{0x3802 ,0x00},
	{0x3803 ,0x00},
	{0x3804 ,0xA },
	{0x3805 ,0x3f},
	{0x3806 ,0x7 },
	{0x3807 ,0x9f},
	{0x3808 ,0x6 },
	{0x3809 ,0x40},
	{0x380a ,0x4 },
	{0x380b ,0xb0},
	{0x380c ,0xc },
	{0x380d ,0x80},
	{0x380e ,0x7 },
	{0x380f ,0xd0},
	{0x5001 ,0xa3},
	{0x5680 ,0x0 },
	{0x5681 ,0x0 },
	{0x5682 ,0xA },
	{0x5683 ,0x20},
	{0x5684 ,0x0 },
	{0x5685 ,0x0 },
	{0x5686 ,0x7 },
	{0x5687 ,0x98},
	{0xffff, 0xff},	
};

//1024x768 XGA
static const struct regval_list OV5640_QSXGA2XGA[]  =	
{
	{0x3800 ,0x00},
	{0x3801 ,0x00},
	{0x3802 ,0x00},
	{0x3803 ,0x00},
	{0x3804 ,0xA },
	{0x3805 ,0x3f},
	{0x3806 ,0x7 },
	{0x3807 ,0x9f},
	{0x3808 ,0x4 },
	{0x3809 ,0x0 },
	{0x380a ,0x3 },
	{0x380b ,0x0 },
	{0x380c ,0xc },
	{0x380d ,0x80},
	{0x380e ,0x7 },
	{0x380f ,0xd0},
	{0x5001 ,0xa3},
	{0x5680 ,0x0 },
	{0x5681 ,0x0 },
	{0x5682 ,0xA },
	{0x5683 ,0x20},
	{0x5684 ,0x0 },
	{0x5685 ,0x0 },
	{0x5686 ,0x7 },
	{0x5687 ,0x98},
	{0xffff, 0xff},	
};

static resolution_t last_res = RES_NONE;

typedef struct dmaControl {
  const stm32_dma_stream_t  *dmastp;
  TIM_TypeDef               *timer;
  uint8_t                   *capture_buffer;
  uint16_t                  dbm_index;
  volatile bool             capture;
  uint32_t                  dma_flags;
  volatile bool             dma_error;
  uint16_t                  dma_count;
} dma_capture_t;

/**
  * Captures an image from the camera.
  * @buffer Buffer in which the image can be sampled
  * @size Size of buffer
  * @res Resolution of the image
  * If resolution MAX_RES has been chosen, the maximum resolution will be
  * chosen for the available buffer. Due to the JPEG compression
  * that could lead to different resolutions on different method calls.
  * The method returns the size of the image.
  */
uint32_t OV5640_Snapshot2RAM(uint8_t* buffer,
                             uint32_t size, resolution_t res) {
	uint8_t cntr = 5;
	//bool status;
	uint32_t size_sampled;

	// Set resolution (if not already done earlier)
	if(res != last_res) {
		if(res == RES_MAX) {
			OV5640_SetResolution(RES_UXGA); // FIXME: We actually have to choose the resolution which fits in the memory
		} else {
			OV5640_SetResolution(res);
		}
	}

	// Capture image until we get a good image or reach max retries.
    TRACE_INFO("CAM  > Capture image into buffer @ 0x%08x size 0x%08x",
               buffer, size);
	do {
		size_sampled = OV5640_Capture(buffer, size);
		if(size_sampled > 0) {
		  TRACE_INFO("CAM  > Image size: %d bytes", size_sampled);
		  return size_sampled;
		}
	} while(cntr--);
    TRACE_ERROR("CAM  > No image captured");
	return 0;
}

//const stm32_dma_stream_t *dmastp;

#if OV5640_USE_DMA_DBM == TRUE

#if !defined(dmaStreamGetCurrentTarget)
/**
 * @brief   Get DMA stream current target.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The stream must have been allocated using @p dmaStreamAllocate().
 * @post    After use the stream can be released using @p dmaStreamRelease().
 *
 * @param[in] dmastp    pointer to a stm32_dma_stream_t structure
 * @return  Current target index
 *
 * @special
 */
#define dmaStreamGetCurrentTarget(dmastp)                                     \
    ((uint8_t)(((dmastp)->stream->CR >> DMA_SxCR_CT_Pos) & 1U))

#endif /* !defined(dmaStreamGetCurrentTarget) */
#endif /* OV5640_USE_DMA_DBM == TRUE */

inline void dma_start(const stm32_dma_stream_t *dmastp) {
  /* Clear any pending interrupts. */
  dmaStreamClearInterrupt(dmastp);
  dmaStreamEnable(dmastp);
}

/*
 * Stop DMA release stream and return count remaining.
 * Note that any DMA FIFO transfer in progress will complete.
 * The Chibios DMAV2 driver waits for EN to clear before proceeding.
 */
inline uint16_t dma_stop(const stm32_dma_stream_t *dmastp) {
	dmaStreamDisable(dmastp);
	uint16_t transfer = dmaStreamGetTransactionSize(dmastp);
	dmaStreamRelease(dmastp);
	return transfer;
}

#if OV5640_USE_DMA_DBM == TRUE

/*
 *
 */
static void dma_interrupt(void *p, uint32_t flags) {
  dma_capture_t *dma_control = p;

  const stm32_dma_stream_t *dmastp = dma_control->dmastp;

  if(flags & (STM32_DMA_ISR_FEIF | STM32_DMA_ISR_TEIF)) {
    /*
     * DMA transfer error or FIFO error.
     * See 9.34.19 of RM0430.
     *
     * Disable DMA and flag fault.
     */
    dma_control->dma_count = dma_stop(dmastp);
    dma_control->dma_error = true;
    dmaStreamClearInterrupt(dmastp);
    return;
  }

  if(flags & STM32_DMA_ISR_HTIF) {
    /*
     * Half transfer complete.
     * Check if DMA is writing to the last buffer.
     */
    if(--dma_control->dbm_index == 0) {
      /*
       * This is the last buffer so we have to terminate DMA.
       * The DBM switch is done in h/w.
       * DMA could write beyond total buffer if not stopped.
       *
       * Since this is the last DMA buffer this is treated as an error.
       * The DMA should normally be terminated by VSYNC before last buffer.
       * Stop DMA and flag error.
       */

      dma_control->dma_count = dma_stop(dmastp);
      dma_control->dma_error = true;
      dmaStreamClearInterrupt(dmastp);
      return;
    }
    /*
     * Else Safe to allow buffer to fill.
     * DMA DBM will switch buffers in h/w when this one is full.
     * Update non-active memory address register now.
     * This is done at TCIF so that CT is known to be valid.
     * Checking CT at TCIF is not good because of IRQ latency.
     * i.e. the DMA controller may have already changed CT before IRQ is serviced.
     */
    dma_control->capture_buffer += DMA_SEGMENT_SIZE;
    if (dmaStreamGetCurrentTarget(dmastp) == 1) {
      dmaStreamSetMemory0(dmastp, dma_control->capture_buffer);
    } else {
      dmaStreamSetMemory1(dmastp, dma_control->capture_buffer);
    }
    dmaStreamClearInterrupt(dmastp);
    return;
  }
  if (flags & STM32_DMA_ISR_TCIF) {
    /*
     * Nothing to do here.
     */
    dmaStreamClearInterrupt(dmastp);
    return;
  }
  /* Unknown IRQ. */
  dmaStreamClearInterrupt(dmastp);
  return;
}

#else

static void dma_interrupt(void *p, uint32_t flags) {
  dma_capture_t *dma_control = p;

  const stm32_dma_stream_t *dmastp = dma_control->dmastp;
  TIM_TypeDef *timer = dma_control->timer;

  dma_control->dma_flags = flags;
  if(flags & (STM32_DMA_ISR_FEIF | STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) {
    /*
     * DMA transfer error, FIFO error or Direct mode error.
     * See 9.34.19 of RM0430.
     */

    dmaStreamClearInterrupt(dma_control->dmastp);
    dma_stop(dma_control->dmastp);
    dma_control->dma_error = true;
    return;
  }

  if((flags & STM32_DMA_ISR_TCIF) != 0) {
    /*
     * If DMA has run to end within a frame then this is an error.
     * In single buffer mode DMA should always be terminated by VSYNC.
     *
     * Disable DMA
     */
    dma_stop(dma_control->dmastp);
    dma_control->dma_error = true;
    return;
  }
  dmaStreamClearInterrupt(dma_control->dmastp);
}

#endif /* USE_OV5640_DMA_DBM */

/*
 * This interrupt handler is for OV5640 compression mode 3.
 * A VSYNC pulse proceeds the JPEG data.
 *
 */
void mode3_vsync_cb(void *arg) {
  dma_capture_t *dma_control = arg;

  const stm32_dma_stream_t *dmastp = dma_control->dmastp;
  TIM_TypeDef *timer = dma_control->timer;

  chSysLockFromISR();
  if(palReadLine(LINE_CAM_VSYNC)) {
    /* VSYNC leading edge. */
    if((timer->DIER & TIM_DIER_CC1DE) != 0) {
      /* Capture running so terminate. */
      dma_control->dma_count = dma_stop(dmastp);
      timer->DIER &= ~TIM_DIER_CC1DE;
      /*
       * Disable VSYNC edge interrupts.
       * Flag image capture complete.
       */
      palDisableLineEventI(LINE_CAM_VSYNC);
      dma_control->capture = true;
    } /* Else wait to arm timer on trailing edge. */
    chSysUnlockFromISR();
    return;
  } /* VSYNC trailing edge. */
  if((timer->DIER & TIM_DIER_CC1DE) == 0) {
    /*
     * Timer not running.
     * Start DMA channel.
     * Enable timer trigger of DMA.
     */
    dma_start(dma_control->dmastp);
    timer->DIER |= TIM_DIER_CC1DE;
  } /* Else wait to stop timer on leading edge. */
  chSysUnlockFromISR();
}

/*
 * VSYNC is asserted to signal frame start.
 * See OV5640 data sheet for details.
 */
static uint8_t vsync_cntr;
void vsync_cb(void *arg) {
  dma_capture_t *dma_control = arg;

  const stm32_dma_stream_t *dmastp = dma_control->dmastp;
  TIM_TypeDef *timer = dma_control->timer;

  chSysLockFromISR();

  // VSYNC handling
  if(!vsync_cntr) {
      /*
       * Rising edge of VSYNC after TIM8 has been initialised.
       * Start DMA channel.
       * Enable TIM8 trigger of DMA.
       */
      dma_start(dmastp);
      timer->DIER |= TIM_DIER_CC1DE;
      vsync_cntr++;
  } else {
      /* VSYNC leading with vsync true.
       * This means end of capture for the frame.
       * Stop & release the DMA channel.
       * Disable TIM8 trigger of DMA.
       * If buffer was filled in DMA then that is an error.
       * We check that here.
       */
      dma_stop(dmastp);
      timer->DIER &= ~TIM_DIER_CC1DE;

      /*
       * Disable VSYNC edge interrupts.
       * Flag image capture complete.
       */
      palDisableLineEventI(LINE_CAM_VSYNC);
      dma_control->capture = true;
  }

  chSysUnlockFromISR();
}

/*
 * Other drivers using resources that can cause DMA competition are locked.
 */
msg_t OV5640_LockResourcesForCapture(void) {
  I2C_Lock();

  msg_t msg = pktAcquireRadio(PKT_RADIO_1, TIME_INFINITE);
  if(msg != MSG_OK) {
    return msg;
  }
  pktPauseReception(PKT_RADIO_1);
  //chMtxLock(&trace_mtx);
  return MSG_OK;
  /* FIXME: USB has to be locked? */
}

/*
 * Unlock competing drivers.
 */
void OV5640_UnlockResourcesForCapture(void) {
  /* FIXME: USB has to be unlocked? */
  //chMtxUnlock(&trace_mtx);
  I2C_Unlock();
  pktResumeReception(PKT_RADIO_1);
  pktReleaseRadio(PKT_RADIO_1);
}

uint32_t OV5640_Capture(uint8_t* buffer, uint32_t size) {

	/*
	 * Note:
	 *  If there are no Chibios devices enabled that use DMA then...
	 *  In makefile add entry to UDEFS:
	 *   UDEFS = -DSTM32_DMA_REQUIRED
	 */

    /* WARNING: Do not use TRACE between locking and unlocking. */
	if(OV5640_LockResourcesForCapture() != MSG_OK) {
	  /* Unable to lock resources. */
	  return 0;
	}

	dma_capture_t dma_control = {0};

	/* Setup DMA for transfer on timer CC tigger.
	 * For TIM8 this DMA2 stream 2, channel 7.
	 * Use PL 3 as PCLCK rate is high.
	 */
	dma_control.dmastp  = STM32_DMA_STREAM(STM32_DMA_STREAM_ID(2, 2));
	uint32_t dmamode = STM32_DMA_CR_CHSEL(7) |
	STM32_DMA_CR_PL(3) |
	STM32_DMA_CR_DIR_P2M |
	STM32_DMA_CR_MSIZE_WORD |
	STM32_DMA_CR_MBURST_INCR4 |
	STM32_DMA_CR_PSIZE_BYTE |
	STM32_DMA_CR_MINC |
	STM32_DMA_CR_DMEIE |
	STM32_DMA_CR_TEIE |
#if OV5640_USE_DMA_DBM == TRUE
    STM32_DMA_CR_DBM |
    STM32_DMA_CR_HTIE |
#endif
	STM32_DMA_CR_TCIE;

	/* Set stream, IRQ priority, IRQ handler & parameter. */
	dmaStreamAllocate(dma_control.dmastp, 7,
	                  (stm32_dmaisr_t)dma_interrupt, &dma_control);

	dmaStreamSetPeripheral(dma_control.dmastp, &GPIOA->IDR); // We want to read the data from here

#if OV5640_USE_DMA_DBM == TRUE
	//dma_buffer = buffer;
	dma_control.capture_buffer = buffer;

    /*
     * Buffer address must be word aligned.
     * Also note requirement for burst transfers from FIFO.
     * A burst write from DMA FIFO to memory must not cross a 1K address boundary.
     * See RM0430 9.3.12
     *
     * TODO: To use DMA_FIFO_BURST_ALIGN in setting of ssdv buffer alignment.
     * Currently this is set to 32 manually in image.c.
     */

    if (((uint32_t)buffer % DMA_FIFO_BURST_ALIGN) != 0) {
      OV5640_UnlockResourcesForCapture();
      TRACE_ERROR("CAM  > Buffer not allocated on DMA burst boundary");
      return 0;
    }
    /*
     * Set the initial buffer addresses.
     * The updating of DMA:MxAR is done in the the DMA interrupt function.
     */
    dmaStreamSetMemory0(dma_control.dmastp, &buffer[0]);
    dmaStreamSetMemory1(dma_control.dmastp, &buffer[DMA_SEGMENT_SIZE]);
    dmaStreamSetTransactionSize(dma_control.dmastp, DMA_SEGMENT_SIZE);

    /*
     * Calculate the number of whole buffers.
     * TODO: Make this include remainder memory as partial buffer?
     */
    dma_control.dbm_index = (size / DMA_SEGMENT_SIZE);
    if(dma_control.dbm_index < 2) {
      OV5640_UnlockResourcesForCapture();
      TRACE_ERROR("CAM  > Capture buffer less than 2 DMA segment segments");
      return 0;
    }
#else
    dmaStreamSetMemory0(dmastp, buffer);
    dmaStreamSetTransactionSize(dma_control.dmastp, size);

#endif
    dmaStreamSetMode(dma_control.dmastp, dmamode); // Setup DMA
    dmaStreamSetFIFO(dma_control.dmastp, STM32_DMA_FCR_DMDIS
                             | STM32_DMA_FCR_FTH_FULL
                             | STM32_DMA_FCR_FEIE);
    dmaStreamClearInterrupt(dma_control.dmastp);

    dma_control.dma_error = false;
    dma_control.dma_flags = 0;

	/*
	 * Setup timer for PCLK
	 */

    dma_control.timer = TIM8;
	rccEnableTIM8(FALSE);
    rccResetTIM8();


	/* TODO: deprecate ARR as not used for Input Capture mode. */
	//TIM8->ARR = 1;
	//dma_control.timer->CCR1 = 0;
	//dma_control.timer->CCER = 0;
    /*
     * Setup capture mode triggered from TI1.
     * What is captured isn't used just the DMA trigger.
     */
	dma_control.timer->CCMR1 = TIM_CCMR1_CC1S_0;
	dma_control.timer->CCER = TIM_CCER_CC1E;

	dma_control.capture = false;
	vsync_cntr = 0;

#define NEW_CAPTURE_VSYNC
#ifdef NEW_CAPTURE_VSYNC
    palSetLineCallback(LINE_CAM_VSYNC, (palcallback_t)mode3_vsync_cb, &dma_control);
    palEnableLineEvent(LINE_CAM_VSYNC, PAL_EVENT_MODE_BOTH_EDGES);
#else
	// Enable VSYNC interrupt
	palSetLineCallback(LINE_CAM_VSYNC, (palcallback_t)vsync_cb, &dma_control);
	palEnableLineEvent(LINE_CAM_VSYNC, PAL_EVENT_MODE_RISING_EDGE);
#endif

	// Wait for capture to be finished
	uint8_t timout = 50; // 500ms max
	do {
		chThdSleep(TIME_MS2I(10));
	} while(!dma_control.capture && !dma_control.dma_error && --timout);

    palDisableLineEvent(LINE_CAM_VSYNC);
    OV5640_UnlockResourcesForCapture();

	if(!timout) {
      TRACE_ERROR("CAM  > Image sampling timeout");
      dma_control.dma_count = dma_stop(dma_control.dmastp);
      dma_control.timer->DIER &= ~TIM_DIER_CC1DE;
      dma_control.dma_error = true;
	}

	if(dma_control.dma_error) {
		if(dma_control.dma_flags & STM32_DMA_ISR_HTIF) {
			TRACE_ERROR("CAM  > DMA abort - last buffer segment");
			error = 0x2;
			return (dma_control.capture_buffer - buffer);
		}
		if(dma_control.dma_flags & STM32_DMA_ISR_FEIF) {
			TRACE_ERROR("CAM  > DMA FIFO error");
			error = 0x3;
		}
		if(dma_control.dma_flags & STM32_DMA_ISR_TEIF) {
			TRACE_ERROR("CAM  > DMA stream transfer error");
			error = 0x4;
		}
		if(dma_control.dma_flags & STM32_DMA_ISR_DMEIF) {
			TRACE_ERROR("CAM  > DMA direct mode error");
			error = 0x5;
		}
		return 0;
	}
    TRACE_INFO("CAM  > Capture success");

	error = 0x0;
#if OV5640_USE_DMA_DBM == TRUE
	return (dma_control.capture_buffer - buffer + (DMA_SEGMENT_SIZE - dma_control.dma_count));
#else
	return size;
#endif
}

/**
  * Initializes GPIO (for pseudo DCMI)
  */
void OV5640_InitGPIO(void)
{
  /* Route to trigger input of timer. */
	palSetLineMode(LINE_CAM_PCLK, PAL_MODE_ALTERNATE(3));
	palSetLineMode(LINE_CAM_VSYNC, PAL_MODE_INPUT | PAL_STM32_OSPEED_HIGHEST);
	palSetLineMode(LINE_CAM_XCLK, PAL_MODE_ALTERNATE(0));
	palSetLineMode(LINE_CAM_D2, PAL_MODE_INPUT | PAL_STM32_OSPEED_HIGHEST);
	palSetLineMode(LINE_CAM_D3, PAL_MODE_INPUT | PAL_STM32_OSPEED_HIGHEST);
	palSetLineMode(LINE_CAM_D4, PAL_MODE_INPUT | PAL_STM32_OSPEED_HIGHEST);
	palSetLineMode(LINE_CAM_D5, PAL_MODE_INPUT | PAL_STM32_OSPEED_HIGHEST);
	palSetLineMode(LINE_CAM_D6, PAL_MODE_INPUT | PAL_STM32_OSPEED_HIGHEST);
	palSetLineMode(LINE_CAM_D7, PAL_MODE_INPUT | PAL_STM32_OSPEED_HIGHEST);
	palSetLineMode(LINE_CAM_D8, PAL_MODE_INPUT | PAL_STM32_OSPEED_HIGHEST);
	palSetLineMode(LINE_CAM_D9, PAL_MODE_INPUT | PAL_STM32_OSPEED_HIGHEST);
	palSetLineMode(LINE_CAM_EN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_CAM_RESET, PAL_MODE_OUTPUT_PUSHPULL);
	/* Force reset low prior to power up. */
	palClearLine(LINE_CAM_RESET);
	chThdSleep(TIME_MS2I(10));
}

void OV5640_TransmitConfig(void)
{
	TRACE_INFO("CAM  > ... Software reset");
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x3103, 0x11);
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x3008, 0x82);
	chThdSleep(TIME_MS2I(100));

	TRACE_INFO("CAM  > ... Initialization");
	for(uint32_t i=0; (OV5640YUV_Sensor_Dvp_Init[i].reg != 0xffff) || (OV5640YUV_Sensor_Dvp_Init[i].val != 0xff); i++)
		I2C_write8_16bitreg(OV5640_I2C_ADR, OV5640YUV_Sensor_Dvp_Init[i].reg, OV5640YUV_Sensor_Dvp_Init[i].val);

	chThdSleep(TIME_MS2I(500));

	TRACE_INFO("CAM  > ... Configure JPEG");
	for(uint32_t i=0; (OV5640_JPEG_QSXGA[i].reg != 0xffff) || (OV5640_JPEG_QSXGA[i].val != 0xff); i++)
		I2C_write8_16bitreg(OV5640_I2C_ADR, OV5640_JPEG_QSXGA[i].reg, OV5640_JPEG_QSXGA[i].val);

	TRACE_INFO("CAM  > ... Light Mode: Auto");
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x3212, 0x03); // start group 3
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x3406, 0x00);
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x3400, 0x04);
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x3401, 0x00);
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x3402, 0x04);
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x3403, 0x00);
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x3404, 0x04);
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x3405, 0x00);
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x3212, 0x13); // end group 3
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x3212, 0xa3); // lanuch group 3
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x5183, 0x0 );

	TRACE_INFO("CAM  > ... Saturation: 0");
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x3212, 0x03); // start group 3
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x5381, 0x1c);
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x5382, 0x5a);
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x5383, 0x06);
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x5384, 0x1a);
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x5385, 0x66);
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x5386, 0x80);
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x5387, 0x82);
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x5388, 0x80);
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x5389, 0x02);
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x538b, 0x98);
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x538a, 0x01);
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x3212, 0x13); // end group 3
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x3212, 0xa3); // launch group 3

	TRACE_INFO("CAM  > ... Brightness: 0");
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x3212, 0x03); // start group 3
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x5587, 0x00);
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x5588, 0x01);
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x3212, 0x13); // end group 3
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x3212, 0xa3); // launch group 3

	TRACE_INFO("CAM  > ... Contrast: 0");
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x3212, 0x03); // start group 3
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x3212, 0x03); // start group 3
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x5586, 0x20);
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x5585, 0x00);
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x3212, 0x13); // end group 3
	I2C_write8_16bitreg(OV5640_I2C_ADR, 0x3212, 0xa3); // launch group 3
}

void OV5640_SetResolution(resolution_t res)
{
	TRACE_INFO("CAM  > ... Configure Resolution");
	switch(res) {
		case RES_QQVGA:
			for(uint32_t i=0; (OV5640_QSXGA2QQVGA[i].reg != 0xffff) || (OV5640_QSXGA2QQVGA[i].val != 0xff); i++)
				I2C_write8_16bitreg(OV5640_I2C_ADR, OV5640_QSXGA2QQVGA[i].reg, OV5640_QSXGA2QQVGA[i].val);
			break;

		case RES_QVGA:
			for(uint32_t i=0; (OV5640_QSXGA2QVGA[i].reg != 0xffff) || (OV5640_QSXGA2QVGA[i].val != 0xff); i++)
				I2C_write8_16bitreg(OV5640_I2C_ADR, OV5640_QSXGA2QVGA[i].reg, OV5640_QSXGA2QVGA[i].val);
			break;

		case RES_VGA:
			for(uint32_t i=0; (OV5640_QSXGA2VGA[i].reg != 0xffff) || (OV5640_QSXGA2VGA[i].val != 0xff); i++)
				I2C_write8_16bitreg(OV5640_I2C_ADR, OV5640_QSXGA2VGA[i].reg, OV5640_QSXGA2VGA[i].val);
			break;

		case RES_XGA:
			for(uint32_t i=0; (OV5640_QSXGA2XGA[i].reg != 0xffff) || (OV5640_QSXGA2XGA[i].val != 0xff); i++)
				I2C_write8_16bitreg(OV5640_I2C_ADR, OV5640_QSXGA2XGA[i].reg, OV5640_QSXGA2XGA[i].val);
			break;

		case RES_UXGA:
			for(uint32_t i=0; (OV5640_QSXGA2UXGA[i].reg != 0xffff) || (OV5640_QSXGA2UXGA[i].val != 0xff); i++)
				I2C_write8_16bitreg(OV5640_I2C_ADR, OV5640_QSXGA2UXGA[i].reg, OV5640_QSXGA2UXGA[i].val);
			break;

		case RES_NONE: // No configuration is made
			break;

		default: // Default QVGA
			for(uint32_t i=0; (OV5640_QSXGA2QVGA[i].reg != 0xffff) || (OV5640_QSXGA2QVGA[i].val != 0xff); i++)
				I2C_write8_16bitreg(OV5640_I2C_ADR, OV5640_QSXGA2QVGA[i].reg, OV5640_QSXGA2QVGA[i].val);
	}
}

void OV5640_powerup(void) {
  // Configure pins
    OV5640_InitGPIO();

    // Switch on camera
    palSetLine(LINE_CAM_EN);        // Switch on camera
    chThdSleep(TIME_MS2I(5));       // Spec is >= 1ms delay after DOVDD stable
    palSetLine(LINE_CAM_RESET);     // Assert reset

    chThdSleep(TIME_MS2I(50));     // Spec is >= 20ms delay after reset high
}


void OV5640_init(void)
{
	TRACE_INFO("CAM  > Init pins");
	OV5640_powerup();
	OV5640_setLightIntensity();

	// Send settings to OV5640
	TRACE_INFO("CAM  > Transmit config to camera");
	OV5640_TransmitConfig();

	chThdSleep(TIME_MS2I(200));
}

void OV5640_deinit(void)
{
	// Power off OV5640
	TRACE_INFO("CAM  > Switch off");

	palSetLineMode(LINE_CAM_PCLK, PAL_MODE_INPUT);
	palSetLineMode(LINE_CAM_VSYNC, PAL_MODE_INPUT);

	palSetLineMode(LINE_CAM_XCLK, PAL_MODE_INPUT);
	palSetLineMode(LINE_CAM_D2, PAL_MODE_INPUT);
	palSetLineMode(LINE_CAM_D3, PAL_MODE_INPUT);
	palSetLineMode(LINE_CAM_D4, PAL_MODE_INPUT);
	palSetLineMode(LINE_CAM_D5, PAL_MODE_INPUT);
	palSetLineMode(LINE_CAM_D6, PAL_MODE_INPUT);
	palSetLineMode(LINE_CAM_D7, PAL_MODE_INPUT);
	palSetLineMode(LINE_CAM_D8, PAL_MODE_INPUT);
	palSetLineMode(LINE_CAM_D9, PAL_MODE_INPUT);

	palSetLineMode(LINE_CAM_EN, PAL_MODE_INPUT);
	palSetLineMode(LINE_CAM_RESET, PAL_MODE_INPUT);

	last_res = RES_NONE;
}

bool OV5640_isAvailable(void) {

  OV5640_powerup();

  uint8_t val, val2;
  bool ret;
  if(I2C_read8_16bitreg(OV5640_I2C_ADR, 0x300A, &val)
      && I2C_read8_16bitreg(OV5640_I2C_ADR, 0x300B, &val2)) {
      ret = val == 0x56 && val2 == 0x40;
  } else {
      error = 0x1;
      ret = false;
  }

  // Switch off camera
  OV5640_deinit();

  return ret;
}

void OV5640_setLightIntensity(void)
{
	uint8_t val1,val2,val3;
	I2C_read8_16bitreg(OV5640_I2C_ADR, 0x3C1B, &val1);
	I2C_read8_16bitreg(OV5640_I2C_ADR, 0x3C1C, &val2);
	I2C_read8_16bitreg(OV5640_I2C_ADR, 0x3C1D, &val3);
	lightIntensity = (val1 << 16) | (val2 << 8) | val3;
}

uint32_t OV5640_getLastLightIntensity(void)
{
	uint32_t ret = lightIntensity;
	return ret;
}

uint8_t OV5640_hasError(void)
{
	return error;
}

