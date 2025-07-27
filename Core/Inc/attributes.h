//#define __GNUC__
/* Check for GNUC */
#if defined (__GNUC__)
	#ifndef __weak		
		#define __weak   	__attribute__((weak))
	#endif	/* Weak attribute */
	#ifndef __packed	
		#define __packed 	__attribute__((__packed__))
	#endif	/* Packed attribute */
#endif
