/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 */
/*! 
 * \file       platform.h
 * \brief        
 *
 * \version    1.0
 * \date       Nov 21 2012
 * \author     Miguel Luis
 */
#ifndef __PLATFORM_H__
#define __PLATFORM_H__


#define bool _Bool
#define true 1
#define false 0

#ifndef __GNUC__
#define inline
#endif

/*!
 * Platform choice. Please uncoment the PLATFORM define and choose your platform
 * or add/change the PLATFORM definition on the compiler Defines option
 */

#define USE_SX1276_RADIO

/*!
 * Module choice. There are three existing module with the SX1276.
 * Please set the connected module to the value 1 and set the others to 0
 */
#ifdef USE_SX1276_RADIO
#define MODULE_SX1276RF1KAS                         1
#endif

#endif // __PLATFORM_H__
