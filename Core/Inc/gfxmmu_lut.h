/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : gfxmmu_lut.h
  * Description        : header file for GFX MMU Configuration Table
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __gfxmmu_lut_H
#define __gfxmmu_lut_H
#ifdef __cplusplus
 extern "C" {
#endif
// GFX MMU Configuration Table

  #define GFXMMU_FB_SIZE 245504
  #define GFXMMU_LUT_FIRST 0
  #define GFXMMU_LUT_LAST  389
  #define GFXMMU_LUT_SIZE 390

uint32_t gfxmmu_lut_config[2*GFXMMU_LUT_SIZE] = {
  0x001A1501, //GFXMMU_LUT0L
  0x003FFEB0, //GFXMMU_LUT0H
  0x001B1401, //GFXMMU_LUT1L
  0x003FFF20, //GFXMMU_LUT1H
  0x001C1401, //GFXMMU_LUT2L
  0x003FFFA0, //GFXMMU_LUT2H
  0x001D1301, //GFXMMU_LUT3L
  0x00000040, //GFXMMU_LUT3H
  0x001D1201, //GFXMMU_LUT4L
  0x00000100, //GFXMMU_LUT4H
  0x001E1201, //GFXMMU_LUT5L
  0x000001C0, //GFXMMU_LUT5H
  0x001E1101, //GFXMMU_LUT6L
  0x000002A0, //GFXMMU_LUT6H
  0x001F1101, //GFXMMU_LUT7L
  0x00000380, //GFXMMU_LUT7H
  0x001F1101, //GFXMMU_LUT8L
  0x00000470, //GFXMMU_LUT8H
  0x00201001, //GFXMMU_LUT9L
  0x00000570, //GFXMMU_LUT9H
  0x00201001, //GFXMMU_LUT10L
  0x00000680, //GFXMMU_LUT10H
  0x00200F01, //GFXMMU_LUT11L
  0x000007A0, //GFXMMU_LUT11H
  0x00210F01, //GFXMMU_LUT12L
  0x000008C0, //GFXMMU_LUT12H
  0x00210F01, //GFXMMU_LUT13L
  0x000009F0, //GFXMMU_LUT13H
  0x00210E01, //GFXMMU_LUT14L
  0x00000B30, //GFXMMU_LUT14H
  0x00220E01, //GFXMMU_LUT15L
  0x00000C70, //GFXMMU_LUT15H
  0x00220E01, //GFXMMU_LUT16L
  0x00000DC0, //GFXMMU_LUT16H
  0x00220E01, //GFXMMU_LUT17L
  0x00000F10, //GFXMMU_LUT17H
  0x00220D01, //GFXMMU_LUT18L
  0x00001070, //GFXMMU_LUT18H
  0x00230D01, //GFXMMU_LUT19L
  0x000011D0, //GFXMMU_LUT19H
  0x00230D01, //GFXMMU_LUT20L
  0x00001340, //GFXMMU_LUT20H
  0x00230D01, //GFXMMU_LUT21L
  0x000014B0, //GFXMMU_LUT21H
  0x00230C01, //GFXMMU_LUT22L
  0x00001630, //GFXMMU_LUT22H
  0x00240C01, //GFXMMU_LUT23L
  0x000017B0, //GFXMMU_LUT23H
  0x00240C01, //GFXMMU_LUT24L
  0x00001940, //GFXMMU_LUT24H
  0x00240C01, //GFXMMU_LUT25L
  0x00001AD0, //GFXMMU_LUT25H
  0x00240B01, //GFXMMU_LUT26L
  0x00001C70, //GFXMMU_LUT26H
  0x00240B01, //GFXMMU_LUT27L
  0x00001E10, //GFXMMU_LUT27H
  0x00250B01, //GFXMMU_LUT28L
  0x00001FB0, //GFXMMU_LUT28H
  0x00250B01, //GFXMMU_LUT29L
  0x00002160, //GFXMMU_LUT29H
  0x00250B01, //GFXMMU_LUT30L
  0x00002310, //GFXMMU_LUT30H
  0x00250A01, //GFXMMU_LUT31L
  0x000024D0, //GFXMMU_LUT31H
  0x00250A01, //GFXMMU_LUT32L
  0x00002690, //GFXMMU_LUT32H
  0x00260A01, //GFXMMU_LUT33L
  0x00002850, //GFXMMU_LUT33H
  0x00260A01, //GFXMMU_LUT34L
  0x00002A20, //GFXMMU_LUT34H
  0x00260A01, //GFXMMU_LUT35L
  0x00002BF0, //GFXMMU_LUT35H
  0x00260A01, //GFXMMU_LUT36L
  0x00002DC0, //GFXMMU_LUT36H
  0x00260901, //GFXMMU_LUT37L
  0x00002FA0, //GFXMMU_LUT37H
  0x00270901, //GFXMMU_LUT38L
  0x00003180, //GFXMMU_LUT38H
  0x00270901, //GFXMMU_LUT39L
  0x00003370, //GFXMMU_LUT39H
  0x00270901, //GFXMMU_LUT40L
  0x00003560, //GFXMMU_LUT40H
  0x00270901, //GFXMMU_LUT41L
  0x00003750, //GFXMMU_LUT41H
  0x00270901, //GFXMMU_LUT42L
  0x00003940, //GFXMMU_LUT42H
  0x00270801, //GFXMMU_LUT43L
  0x00003B40, //GFXMMU_LUT43H
  0x00270801, //GFXMMU_LUT44L
  0x00003D40, //GFXMMU_LUT44H
  0x00280801, //GFXMMU_LUT45L
  0x00003F40, //GFXMMU_LUT45H
  0x00280801, //GFXMMU_LUT46L
  0x00004150, //GFXMMU_LUT46H
  0x00280801, //GFXMMU_LUT47L
  0x00004360, //GFXMMU_LUT47H
  0x00280801, //GFXMMU_LUT48L
  0x00004570, //GFXMMU_LUT48H
  0x00280801, //GFXMMU_LUT49L
  0x00004780, //GFXMMU_LUT49H
  0x00280701, //GFXMMU_LUT50L
  0x000049A0, //GFXMMU_LUT50H
  0x00280701, //GFXMMU_LUT51L
  0x00004BC0, //GFXMMU_LUT51H
  0x00290701, //GFXMMU_LUT52L
  0x00004DE0, //GFXMMU_LUT52H
  0x00290701, //GFXMMU_LUT53L
  0x00005010, //GFXMMU_LUT53H
  0x00290701, //GFXMMU_LUT54L
  0x00005240, //GFXMMU_LUT54H
  0x00290701, //GFXMMU_LUT55L
  0x00005470, //GFXMMU_LUT55H
  0x00290701, //GFXMMU_LUT56L
  0x000056A0, //GFXMMU_LUT56H
  0x00290701, //GFXMMU_LUT57L
  0x000058D0, //GFXMMU_LUT57H
  0x00290601, //GFXMMU_LUT58L
  0x00005B10, //GFXMMU_LUT58H
  0x00290601, //GFXMMU_LUT59L
  0x00005D50, //GFXMMU_LUT59H
  0x002A0601, //GFXMMU_LUT60L
  0x00005F90, //GFXMMU_LUT60H
  0x002A0601, //GFXMMU_LUT61L
  0x000061E0, //GFXMMU_LUT61H
  0x002A0601, //GFXMMU_LUT62L
  0x00006430, //GFXMMU_LUT62H
  0x002A0601, //GFXMMU_LUT63L
  0x00006680, //GFXMMU_LUT63H
  0x002A0601, //GFXMMU_LUT64L
  0x000068D0, //GFXMMU_LUT64H
  0x002A0601, //GFXMMU_LUT65L
  0x00006B20, //GFXMMU_LUT65H
  0x002A0501, //GFXMMU_LUT66L
  0x00006D80, //GFXMMU_LUT66H
  0x002A0501, //GFXMMU_LUT67L
  0x00006FE0, //GFXMMU_LUT67H
  0x002A0501, //GFXMMU_LUT68L
  0x00007240, //GFXMMU_LUT68H
  0x002B0501, //GFXMMU_LUT69L
  0x000074A0, //GFXMMU_LUT69H
  0x002B0501, //GFXMMU_LUT70L
  0x00007710, //GFXMMU_LUT70H
  0x002B0501, //GFXMMU_LUT71L
  0x00007980, //GFXMMU_LUT71H
  0x002B0501, //GFXMMU_LUT72L
  0x00007BF0, //GFXMMU_LUT72H
  0x002B0501, //GFXMMU_LUT73L
  0x00007E60, //GFXMMU_LUT73H
  0x002B0501, //GFXMMU_LUT74L
  0x000080D0, //GFXMMU_LUT74H
  0x002B0501, //GFXMMU_LUT75L
  0x00008340, //GFXMMU_LUT75H
  0x002B0401, //GFXMMU_LUT76L
  0x000085C0, //GFXMMU_LUT76H
  0x002B0401, //GFXMMU_LUT77L
  0x00008840, //GFXMMU_LUT77H
  0x002B0401, //GFXMMU_LUT78L
  0x00008AC0, //GFXMMU_LUT78H
  0x002C0401, //GFXMMU_LUT79L
  0x00008D40, //GFXMMU_LUT79H
  0x002C0401, //GFXMMU_LUT80L
  0x00008FD0, //GFXMMU_LUT80H
  0x002C0401, //GFXMMU_LUT81L
  0x00009260, //GFXMMU_LUT81H
  0x002C0401, //GFXMMU_LUT82L
  0x000094F0, //GFXMMU_LUT82H
  0x002C0401, //GFXMMU_LUT83L
  0x00009780, //GFXMMU_LUT83H
  0x002C0401, //GFXMMU_LUT84L
  0x00009A10, //GFXMMU_LUT84H
  0x002C0401, //GFXMMU_LUT85L
  0x00009CA0, //GFXMMU_LUT85H
  0x002C0401, //GFXMMU_LUT86L
  0x00009F30, //GFXMMU_LUT86H
  0x002C0301, //GFXMMU_LUT87L
  0x0000A1D0, //GFXMMU_LUT87H
  0x002C0301, //GFXMMU_LUT88L
  0x0000A470, //GFXMMU_LUT88H
  0x002C0301, //GFXMMU_LUT89L
  0x0000A710, //GFXMMU_LUT89H
  0x002C0301, //GFXMMU_LUT90L
  0x0000A9B0, //GFXMMU_LUT90H
  0x002D0301, //GFXMMU_LUT91L
  0x0000AC50, //GFXMMU_LUT91H
  0x002D0301, //GFXMMU_LUT92L
  0x0000AF00, //GFXMMU_LUT92H
  0x002D0301, //GFXMMU_LUT93L
  0x0000B1B0, //GFXMMU_LUT93H
  0x002D0301, //GFXMMU_LUT94L
  0x0000B460, //GFXMMU_LUT94H
  0x002D0301, //GFXMMU_LUT95L
  0x0000B710, //GFXMMU_LUT95H
  0x002D0301, //GFXMMU_LUT96L
  0x0000B9C0, //GFXMMU_LUT96H
  0x002D0301, //GFXMMU_LUT97L
  0x0000BC70, //GFXMMU_LUT97H
  0x002D0301, //GFXMMU_LUT98L
  0x0000BF20, //GFXMMU_LUT98H
  0x002D0301, //GFXMMU_LUT99L
  0x0000C1D0, //GFXMMU_LUT99H
  0x002D0301, //GFXMMU_LUT100L
  0x0000C480, //GFXMMU_LUT100H
  0x002D0201, //GFXMMU_LUT101L
  0x0000C740, //GFXMMU_LUT101H
  0x002D0201, //GFXMMU_LUT102L
  0x0000CA00, //GFXMMU_LUT102H
  0x002D0201, //GFXMMU_LUT103L
  0x0000CCC0, //GFXMMU_LUT103H
  0x002D0201, //GFXMMU_LUT104L
  0x0000CF80, //GFXMMU_LUT104H
  0x002E0201, //GFXMMU_LUT105L
  0x0000D240, //GFXMMU_LUT105H
  0x002E0201, //GFXMMU_LUT106L
  0x0000D510, //GFXMMU_LUT106H
  0x002E0201, //GFXMMU_LUT107L
  0x0000D7E0, //GFXMMU_LUT107H
  0x002E0201, //GFXMMU_LUT108L
  0x0000DAB0, //GFXMMU_LUT108H
  0x002E0201, //GFXMMU_LUT109L
  0x0000DD80, //GFXMMU_LUT109H
  0x002E0201, //GFXMMU_LUT110L
  0x0000E050, //GFXMMU_LUT110H
  0x002E0201, //GFXMMU_LUT111L
  0x0000E320, //GFXMMU_LUT111H
  0x002E0201, //GFXMMU_LUT112L
  0x0000E5F0, //GFXMMU_LUT112H
  0x002E0201, //GFXMMU_LUT113L
  0x0000E8C0, //GFXMMU_LUT113H
  0x002E0201, //GFXMMU_LUT114L
  0x0000EB90, //GFXMMU_LUT114H
  0x002E0201, //GFXMMU_LUT115L
  0x0000EE60, //GFXMMU_LUT115H
  0x002E0201, //GFXMMU_LUT116L
  0x0000F130, //GFXMMU_LUT116H
  0x002E0101, //GFXMMU_LUT117L
  0x0000F410, //GFXMMU_LUT117H
  0x002E0101, //GFXMMU_LUT118L
  0x0000F6F0, //GFXMMU_LUT118H
  0x002E0101, //GFXMMU_LUT119L
  0x0000F9D0, //GFXMMU_LUT119H
  0x002E0101, //GFXMMU_LUT120L
  0x0000FCB0, //GFXMMU_LUT120H
  0x002E0101, //GFXMMU_LUT121L
  0x0000FF90, //GFXMMU_LUT121H
  0x002F0101, //GFXMMU_LUT122L
  0x00010270, //GFXMMU_LUT122H
  0x002F0101, //GFXMMU_LUT123L
  0x00010560, //GFXMMU_LUT123H
  0x002F0101, //GFXMMU_LUT124L
  0x00010850, //GFXMMU_LUT124H
  0x002F0101, //GFXMMU_LUT125L
  0x00010B40, //GFXMMU_LUT125H
  0x002F0101, //GFXMMU_LUT126L
  0x00010E30, //GFXMMU_LUT126H
  0x002F0101, //GFXMMU_LUT127L
  0x00011120, //GFXMMU_LUT127H
  0x002F0101, //GFXMMU_LUT128L
  0x00011410, //GFXMMU_LUT128H
  0x002F0101, //GFXMMU_LUT129L
  0x00011700, //GFXMMU_LUT129H
  0x002F0101, //GFXMMU_LUT130L
  0x000119F0, //GFXMMU_LUT130H
  0x002F0101, //GFXMMU_LUT131L
  0x00011CE0, //GFXMMU_LUT131H
  0x002F0101, //GFXMMU_LUT132L
  0x00011FD0, //GFXMMU_LUT132H
  0x002F0101, //GFXMMU_LUT133L
  0x000122C0, //GFXMMU_LUT133H
  0x002F0101, //GFXMMU_LUT134L
  0x000125B0, //GFXMMU_LUT134H
  0x002F0101, //GFXMMU_LUT135L
  0x000128A0, //GFXMMU_LUT135H
  0x002F0101, //GFXMMU_LUT136L
  0x00012B90, //GFXMMU_LUT136H
  0x002F0101, //GFXMMU_LUT137L
  0x00012E80, //GFXMMU_LUT137H
  0x002F0101, //GFXMMU_LUT138L
  0x00013170, //GFXMMU_LUT138H
  0x002F0001, //GFXMMU_LUT139L
  0x00013470, //GFXMMU_LUT139H
  0x002F0001, //GFXMMU_LUT140L
  0x00013770, //GFXMMU_LUT140H
  0x002F0001, //GFXMMU_LUT141L
  0x00013A70, //GFXMMU_LUT141H
  0x002F0001, //GFXMMU_LUT142L
  0x00013D70, //GFXMMU_LUT142H
  0x002F0001, //GFXMMU_LUT143L
  0x00014070, //GFXMMU_LUT143H
  0x002F0001, //GFXMMU_LUT144L
  0x00014370, //GFXMMU_LUT144H
  0x002F0001, //GFXMMU_LUT145L
  0x00014670, //GFXMMU_LUT145H
  0x00300001, //GFXMMU_LUT146L
  0x00014970, //GFXMMU_LUT146H
  0x00300001, //GFXMMU_LUT147L
  0x00014C80, //GFXMMU_LUT147H
  0x00300001, //GFXMMU_LUT148L
  0x00014F90, //GFXMMU_LUT148H
  0x00300001, //GFXMMU_LUT149L
  0x000152A0, //GFXMMU_LUT149H
  0x00300001, //GFXMMU_LUT150L
  0x000155B0, //GFXMMU_LUT150H
  0x00300001, //GFXMMU_LUT151L
  0x000158C0, //GFXMMU_LUT151H
  0x00300001, //GFXMMU_LUT152L
  0x00015BD0, //GFXMMU_LUT152H
  0x00300001, //GFXMMU_LUT153L
  0x00015EE0, //GFXMMU_LUT153H
  0x00300001, //GFXMMU_LUT154L
  0x000161F0, //GFXMMU_LUT154H
  0x00300001, //GFXMMU_LUT155L
  0x00016500, //GFXMMU_LUT155H
  0x00300001, //GFXMMU_LUT156L
  0x00016810, //GFXMMU_LUT156H
  0x00300001, //GFXMMU_LUT157L
  0x00016B20, //GFXMMU_LUT157H
  0x00300001, //GFXMMU_LUT158L
  0x00016E30, //GFXMMU_LUT158H
  0x00300001, //GFXMMU_LUT159L
  0x00017140, //GFXMMU_LUT159H
  0x00300001, //GFXMMU_LUT160L
  0x00017450, //GFXMMU_LUT160H
  0x00300001, //GFXMMU_LUT161L
  0x00017760, //GFXMMU_LUT161H
  0x00300001, //GFXMMU_LUT162L
  0x00017A70, //GFXMMU_LUT162H
  0x00300001, //GFXMMU_LUT163L
  0x00017D80, //GFXMMU_LUT163H
  0x00300001, //GFXMMU_LUT164L
  0x00018090, //GFXMMU_LUT164H
  0x00300001, //GFXMMU_LUT165L
  0x000183A0, //GFXMMU_LUT165H
  0x00300001, //GFXMMU_LUT166L
  0x000186B0, //GFXMMU_LUT166H
  0x00300001, //GFXMMU_LUT167L
  0x000189C0, //GFXMMU_LUT167H
  0x00300001, //GFXMMU_LUT168L
  0x00018CD0, //GFXMMU_LUT168H
  0x00300001, //GFXMMU_LUT169L
  0x00018FE0, //GFXMMU_LUT169H
  0x00300001, //GFXMMU_LUT170L
  0x000192F0, //GFXMMU_LUT170H
  0x00300001, //GFXMMU_LUT171L
  0x00019600, //GFXMMU_LUT171H
  0x00300001, //GFXMMU_LUT172L
  0x00019910, //GFXMMU_LUT172H
  0x00300001, //GFXMMU_LUT173L
  0x00019C20, //GFXMMU_LUT173H
  0x00300001, //GFXMMU_LUT174L
  0x00019F30, //GFXMMU_LUT174H
  0x00300001, //GFXMMU_LUT175L
  0x0001A240, //GFXMMU_LUT175H
  0x00300001, //GFXMMU_LUT176L
  0x0001A550, //GFXMMU_LUT176H
  0x00300001, //GFXMMU_LUT177L
  0x0001A860, //GFXMMU_LUT177H
  0x00300001, //GFXMMU_LUT178L
  0x0001AB70, //GFXMMU_LUT178H
  0x00300001, //GFXMMU_LUT179L
  0x0001AE80, //GFXMMU_LUT179H
  0x00300001, //GFXMMU_LUT180L
  0x0001B190, //GFXMMU_LUT180H
  0x00300001, //GFXMMU_LUT181L
  0x0001B4A0, //GFXMMU_LUT181H
  0x00300001, //GFXMMU_LUT182L
  0x0001B7B0, //GFXMMU_LUT182H
  0x00300001, //GFXMMU_LUT183L
  0x0001BAC0, //GFXMMU_LUT183H
  0x00300001, //GFXMMU_LUT184L
  0x0001BDD0, //GFXMMU_LUT184H
  0x00300001, //GFXMMU_LUT185L
  0x0001C0E0, //GFXMMU_LUT185H
  0x00300001, //GFXMMU_LUT186L
  0x0001C3F0, //GFXMMU_LUT186H
  0x00300001, //GFXMMU_LUT187L
  0x0001C700, //GFXMMU_LUT187H
  0x00300001, //GFXMMU_LUT188L
  0x0001CA10, //GFXMMU_LUT188H
  0x00300001, //GFXMMU_LUT189L
  0x0001CD20, //GFXMMU_LUT189H
  0x00300001, //GFXMMU_LUT190L
  0x0001D030, //GFXMMU_LUT190H
  0x00300001, //GFXMMU_LUT191L
  0x0001D340, //GFXMMU_LUT191H
  0x00300001, //GFXMMU_LUT192L
  0x0001D650, //GFXMMU_LUT192H
  0x00300001, //GFXMMU_LUT193L
  0x0001D960, //GFXMMU_LUT193H
  0x00300001, //GFXMMU_LUT194L
  0x0001DC70, //GFXMMU_LUT194H
  0x00300001, //GFXMMU_LUT195L
  0x0001DF80, //GFXMMU_LUT195H
  0x00300001, //GFXMMU_LUT196L
  0x0001E290, //GFXMMU_LUT196H
  0x00300001, //GFXMMU_LUT197L
  0x0001E5A0, //GFXMMU_LUT197H
  0x00300001, //GFXMMU_LUT198L
  0x0001E8B0, //GFXMMU_LUT198H
  0x00300001, //GFXMMU_LUT199L
  0x0001EBC0, //GFXMMU_LUT199H
  0x00300001, //GFXMMU_LUT200L
  0x0001EED0, //GFXMMU_LUT200H
  0x00300001, //GFXMMU_LUT201L
  0x0001F1E0, //GFXMMU_LUT201H
  0x00300001, //GFXMMU_LUT202L
  0x0001F4F0, //GFXMMU_LUT202H
  0x00300001, //GFXMMU_LUT203L
  0x0001F800, //GFXMMU_LUT203H
  0x00300001, //GFXMMU_LUT204L
  0x0001FB10, //GFXMMU_LUT204H
  0x00300001, //GFXMMU_LUT205L
  0x0001FE20, //GFXMMU_LUT205H
  0x00300001, //GFXMMU_LUT206L
  0x00020130, //GFXMMU_LUT206H
  0x00300001, //GFXMMU_LUT207L
  0x00020440, //GFXMMU_LUT207H
  0x00300001, //GFXMMU_LUT208L
  0x00020750, //GFXMMU_LUT208H
  0x00300001, //GFXMMU_LUT209L
  0x00020A60, //GFXMMU_LUT209H
  0x00300001, //GFXMMU_LUT210L
  0x00020D70, //GFXMMU_LUT210H
  0x00300001, //GFXMMU_LUT211L
  0x00021080, //GFXMMU_LUT211H
  0x00300001, //GFXMMU_LUT212L
  0x00021390, //GFXMMU_LUT212H
  0x00300001, //GFXMMU_LUT213L
  0x000216A0, //GFXMMU_LUT213H
  0x00300001, //GFXMMU_LUT214L
  0x000219B0, //GFXMMU_LUT214H
  0x00300001, //GFXMMU_LUT215L
  0x00021CC0, //GFXMMU_LUT215H
  0x00300001, //GFXMMU_LUT216L
  0x00021FD0, //GFXMMU_LUT216H
  0x00300001, //GFXMMU_LUT217L
  0x000222E0, //GFXMMU_LUT217H
  0x00300001, //GFXMMU_LUT218L
  0x000225F0, //GFXMMU_LUT218H
  0x00300001, //GFXMMU_LUT219L
  0x00022900, //GFXMMU_LUT219H
  0x00300001, //GFXMMU_LUT220L
  0x00022C10, //GFXMMU_LUT220H
  0x00300001, //GFXMMU_LUT221L
  0x00022F20, //GFXMMU_LUT221H
  0x00300001, //GFXMMU_LUT222L
  0x00023230, //GFXMMU_LUT222H
  0x00300001, //GFXMMU_LUT223L
  0x00023540, //GFXMMU_LUT223H
  0x00300001, //GFXMMU_LUT224L
  0x00023850, //GFXMMU_LUT224H
  0x00300001, //GFXMMU_LUT225L
  0x00023B60, //GFXMMU_LUT225H
  0x00300001, //GFXMMU_LUT226L
  0x00023E70, //GFXMMU_LUT226H
  0x00300001, //GFXMMU_LUT227L
  0x00024180, //GFXMMU_LUT227H
  0x00300001, //GFXMMU_LUT228L
  0x00024490, //GFXMMU_LUT228H
  0x00300001, //GFXMMU_LUT229L
  0x000247A0, //GFXMMU_LUT229H
  0x00300001, //GFXMMU_LUT230L
  0x00024AB0, //GFXMMU_LUT230H
  0x00300001, //GFXMMU_LUT231L
  0x00024DC0, //GFXMMU_LUT231H
  0x00300001, //GFXMMU_LUT232L
  0x000250D0, //GFXMMU_LUT232H
  0x00300001, //GFXMMU_LUT233L
  0x000253E0, //GFXMMU_LUT233H
  0x00300001, //GFXMMU_LUT234L
  0x000256F0, //GFXMMU_LUT234H
  0x00300001, //GFXMMU_LUT235L
  0x00025A00, //GFXMMU_LUT235H
  0x00300001, //GFXMMU_LUT236L
  0x00025D10, //GFXMMU_LUT236H
  0x00300001, //GFXMMU_LUT237L
  0x00026020, //GFXMMU_LUT237H
  0x00300001, //GFXMMU_LUT238L
  0x00026330, //GFXMMU_LUT238H
  0x00300001, //GFXMMU_LUT239L
  0x00026640, //GFXMMU_LUT239H
  0x00300001, //GFXMMU_LUT240L
  0x00026950, //GFXMMU_LUT240H
  0x00300001, //GFXMMU_LUT241L
  0x00026C60, //GFXMMU_LUT241H
  0x00300001, //GFXMMU_LUT242L
  0x00026F70, //GFXMMU_LUT242H
  0x00300001, //GFXMMU_LUT243L
  0x00027280, //GFXMMU_LUT243H
  0x002F0001, //GFXMMU_LUT244L
  0x00027590, //GFXMMU_LUT244H
  0x002F0001, //GFXMMU_LUT245L
  0x00027890, //GFXMMU_LUT245H
  0x002F0001, //GFXMMU_LUT246L
  0x00027B90, //GFXMMU_LUT246H
  0x002F0001, //GFXMMU_LUT247L
  0x00027E90, //GFXMMU_LUT247H
  0x002F0001, //GFXMMU_LUT248L
  0x00028190, //GFXMMU_LUT248H
  0x002F0001, //GFXMMU_LUT249L
  0x00028490, //GFXMMU_LUT249H
  0x002F0001, //GFXMMU_LUT250L
  0x00028790, //GFXMMU_LUT250H
  0x002F0101, //GFXMMU_LUT251L
  0x00028A80, //GFXMMU_LUT251H
  0x002F0101, //GFXMMU_LUT252L
  0x00028D70, //GFXMMU_LUT252H
  0x002F0101, //GFXMMU_LUT253L
  0x00029060, //GFXMMU_LUT253H
  0x002F0101, //GFXMMU_LUT254L
  0x00029350, //GFXMMU_LUT254H
  0x002F0101, //GFXMMU_LUT255L
  0x00029640, //GFXMMU_LUT255H
  0x002F0101, //GFXMMU_LUT256L
  0x00029930, //GFXMMU_LUT256H
  0x002F0101, //GFXMMU_LUT257L
  0x00029C20, //GFXMMU_LUT257H
  0x002F0101, //GFXMMU_LUT258L
  0x00029F10, //GFXMMU_LUT258H
  0x002F0101, //GFXMMU_LUT259L
  0x0002A200, //GFXMMU_LUT259H
  0x002F0101, //GFXMMU_LUT260L
  0x0002A4F0, //GFXMMU_LUT260H
  0x002F0101, //GFXMMU_LUT261L
  0x0002A7E0, //GFXMMU_LUT261H
  0x002F0101, //GFXMMU_LUT262L
  0x0002AAD0, //GFXMMU_LUT262H
  0x002F0101, //GFXMMU_LUT263L
  0x0002ADC0, //GFXMMU_LUT263H
  0x002F0101, //GFXMMU_LUT264L
  0x0002B0B0, //GFXMMU_LUT264H
  0x002F0101, //GFXMMU_LUT265L
  0x0002B3A0, //GFXMMU_LUT265H
  0x002F0101, //GFXMMU_LUT266L
  0x0002B690, //GFXMMU_LUT266H
  0x002F0101, //GFXMMU_LUT267L
  0x0002B980, //GFXMMU_LUT267H
  0x002E0101, //GFXMMU_LUT268L
  0x0002BC70, //GFXMMU_LUT268H
  0x002E0101, //GFXMMU_LUT269L
  0x0002BF50, //GFXMMU_LUT269H
  0x002E0101, //GFXMMU_LUT270L
  0x0002C230, //GFXMMU_LUT270H
  0x002E0101, //GFXMMU_LUT271L
  0x0002C510, //GFXMMU_LUT271H
  0x002E0101, //GFXMMU_LUT272L
  0x0002C7F0, //GFXMMU_LUT272H
  0x002E0201, //GFXMMU_LUT273L
  0x0002CAC0, //GFXMMU_LUT273H
  0x002E0201, //GFXMMU_LUT274L
  0x0002CD90, //GFXMMU_LUT274H
  0x002E0201, //GFXMMU_LUT275L
  0x0002D060, //GFXMMU_LUT275H
  0x002E0201, //GFXMMU_LUT276L
  0x0002D330, //GFXMMU_LUT276H
  0x002E0201, //GFXMMU_LUT277L
  0x0002D600, //GFXMMU_LUT277H
  0x002E0201, //GFXMMU_LUT278L
  0x0002D8D0, //GFXMMU_LUT278H
  0x002E0201, //GFXMMU_LUT279L
  0x0002DBA0, //GFXMMU_LUT279H
  0x002E0201, //GFXMMU_LUT280L
  0x0002DE70, //GFXMMU_LUT280H
  0x002E0201, //GFXMMU_LUT281L
  0x0002E140, //GFXMMU_LUT281H
  0x002E0201, //GFXMMU_LUT282L
  0x0002E410, //GFXMMU_LUT282H
  0x002E0201, //GFXMMU_LUT283L
  0x0002E6E0, //GFXMMU_LUT283H
  0x002E0201, //GFXMMU_LUT284L
  0x0002E9B0, //GFXMMU_LUT284H
  0x002D0201, //GFXMMU_LUT285L
  0x0002EC80, //GFXMMU_LUT285H
  0x002D0201, //GFXMMU_LUT286L
  0x0002EF40, //GFXMMU_LUT286H
  0x002D0201, //GFXMMU_LUT287L
  0x0002F200, //GFXMMU_LUT287H
  0x002D0201, //GFXMMU_LUT288L
  0x0002F4C0, //GFXMMU_LUT288H
  0x002D0301, //GFXMMU_LUT289L
  0x0002F770, //GFXMMU_LUT289H
  0x002D0301, //GFXMMU_LUT290L
  0x0002FA20, //GFXMMU_LUT290H
  0x002D0301, //GFXMMU_LUT291L
  0x0002FCD0, //GFXMMU_LUT291H
  0x002D0301, //GFXMMU_LUT292L
  0x0002FF80, //GFXMMU_LUT292H
  0x002D0301, //GFXMMU_LUT293L
  0x00030230, //GFXMMU_LUT293H
  0x002D0301, //GFXMMU_LUT294L
  0x000304E0, //GFXMMU_LUT294H
  0x002D0301, //GFXMMU_LUT295L
  0x00030790, //GFXMMU_LUT295H
  0x002D0301, //GFXMMU_LUT296L
  0x00030A40, //GFXMMU_LUT296H
  0x002D0301, //GFXMMU_LUT297L
  0x00030CF0, //GFXMMU_LUT297H
  0x002D0301, //GFXMMU_LUT298L
  0x00030FA0, //GFXMMU_LUT298H
  0x002C0301, //GFXMMU_LUT299L
  0x00031250, //GFXMMU_LUT299H
  0x002C0301, //GFXMMU_LUT300L
  0x000314F0, //GFXMMU_LUT300H
  0x002C0301, //GFXMMU_LUT301L
  0x00031790, //GFXMMU_LUT301H
  0x002C0301, //GFXMMU_LUT302L
  0x00031A30, //GFXMMU_LUT302H
  0x002C0401, //GFXMMU_LUT303L
  0x00031CC0, //GFXMMU_LUT303H
  0x002C0401, //GFXMMU_LUT304L
  0x00031F50, //GFXMMU_LUT304H
  0x002C0401, //GFXMMU_LUT305L
  0x000321E0, //GFXMMU_LUT305H
  0x002C0401, //GFXMMU_LUT306L
  0x00032470, //GFXMMU_LUT306H
  0x002C0401, //GFXMMU_LUT307L
  0x00032700, //GFXMMU_LUT307H
  0x002C0401, //GFXMMU_LUT308L
  0x00032990, //GFXMMU_LUT308H
  0x002C0401, //GFXMMU_LUT309L
  0x00032C20, //GFXMMU_LUT309H
  0x002C0401, //GFXMMU_LUT310L
  0x00032EB0, //GFXMMU_LUT310H
  0x002B0401, //GFXMMU_LUT311L
  0x00033140, //GFXMMU_LUT311H
  0x002B0401, //GFXMMU_LUT312L
  0x000333C0, //GFXMMU_LUT312H
  0x002B0401, //GFXMMU_LUT313L
  0x00033640, //GFXMMU_LUT313H
  0x002B0501, //GFXMMU_LUT314L
  0x000338B0, //GFXMMU_LUT314H
  0x002B0501, //GFXMMU_LUT315L
  0x00033B20, //GFXMMU_LUT315H
  0x002B0501, //GFXMMU_LUT316L
  0x00033D90, //GFXMMU_LUT316H
  0x002B0501, //GFXMMU_LUT317L
  0x00034000, //GFXMMU_LUT317H
  0x002B0501, //GFXMMU_LUT318L
  0x00034270, //GFXMMU_LUT318H
  0x002B0501, //GFXMMU_LUT319L
  0x000344E0, //GFXMMU_LUT319H
  0x002B0501, //GFXMMU_LUT320L
  0x00034750, //GFXMMU_LUT320H
  0x002A0501, //GFXMMU_LUT321L
  0x000349C0, //GFXMMU_LUT321H
  0x002A0501, //GFXMMU_LUT322L
  0x00034C20, //GFXMMU_LUT322H
  0x002A0501, //GFXMMU_LUT323L
  0x00034E80, //GFXMMU_LUT323H
  0x002A0601, //GFXMMU_LUT324L
  0x000350D0, //GFXMMU_LUT324H
  0x002A0601, //GFXMMU_LUT325L
  0x00035320, //GFXMMU_LUT325H
  0x002A0601, //GFXMMU_LUT326L
  0x00035570, //GFXMMU_LUT326H
  0x002A0601, //GFXMMU_LUT327L
  0x000357C0, //GFXMMU_LUT327H
  0x002A0601, //GFXMMU_LUT328L
  0x00035A10, //GFXMMU_LUT328H
  0x002A0601, //GFXMMU_LUT329L
  0x00035C60, //GFXMMU_LUT329H
  0x00290601, //GFXMMU_LUT330L
  0x00035EB0, //GFXMMU_LUT330H
  0x00290601, //GFXMMU_LUT331L
  0x000360F0, //GFXMMU_LUT331H
  0x00290701, //GFXMMU_LUT332L
  0x00036320, //GFXMMU_LUT332H
  0x00290701, //GFXMMU_LUT333L
  0x00036550, //GFXMMU_LUT333H
  0x00290701, //GFXMMU_LUT334L
  0x00036780, //GFXMMU_LUT334H
  0x00290701, //GFXMMU_LUT335L
  0x000369B0, //GFXMMU_LUT335H
  0x00290701, //GFXMMU_LUT336L
  0x00036BE0, //GFXMMU_LUT336H
  0x00290701, //GFXMMU_LUT337L
  0x00036E10, //GFXMMU_LUT337H
  0x00280701, //GFXMMU_LUT338L
  0x00037040, //GFXMMU_LUT338H
  0x00280701, //GFXMMU_LUT339L
  0x00037260, //GFXMMU_LUT339H
  0x00280801, //GFXMMU_LUT340L
  0x00037470, //GFXMMU_LUT340H
  0x00280801, //GFXMMU_LUT341L
  0x00037680, //GFXMMU_LUT341H
  0x00280801, //GFXMMU_LUT342L
  0x00037890, //GFXMMU_LUT342H
  0x00280801, //GFXMMU_LUT343L
  0x00037AA0, //GFXMMU_LUT343H
  0x00280801, //GFXMMU_LUT344L
  0x00037CB0, //GFXMMU_LUT344H
  0x00270801, //GFXMMU_LUT345L
  0x00037EC0, //GFXMMU_LUT345H
  0x00270801, //GFXMMU_LUT346L
  0x000380C0, //GFXMMU_LUT346H
  0x00270901, //GFXMMU_LUT347L
  0x000382B0, //GFXMMU_LUT347H
  0x00270901, //GFXMMU_LUT348L
  0x000384A0, //GFXMMU_LUT348H
  0x00270901, //GFXMMU_LUT349L
  0x00038690, //GFXMMU_LUT349H
  0x00270901, //GFXMMU_LUT350L
  0x00038880, //GFXMMU_LUT350H
  0x00270901, //GFXMMU_LUT351L
  0x00038A70, //GFXMMU_LUT351H
  0x00260901, //GFXMMU_LUT352L
  0x00038C60, //GFXMMU_LUT352H
  0x00260A01, //GFXMMU_LUT353L
  0x00038E30, //GFXMMU_LUT353H
  0x00260A01, //GFXMMU_LUT354L
  0x00039000, //GFXMMU_LUT354H
  0x00260A01, //GFXMMU_LUT355L
  0x000391D0, //GFXMMU_LUT355H
  0x00260A01, //GFXMMU_LUT356L
  0x000393A0, //GFXMMU_LUT356H
  0x00250A01, //GFXMMU_LUT357L
  0x00039570, //GFXMMU_LUT357H
  0x00250A01, //GFXMMU_LUT358L
  0x00039730, //GFXMMU_LUT358H
  0x00250B01, //GFXMMU_LUT359L
  0x000398E0, //GFXMMU_LUT359H
  0x00250B01, //GFXMMU_LUT360L
  0x00039A90, //GFXMMU_LUT360H
  0x00250B01, //GFXMMU_LUT361L
  0x00039C40, //GFXMMU_LUT361H
  0x00240B01, //GFXMMU_LUT362L
  0x00039DF0, //GFXMMU_LUT362H
  0x00240B01, //GFXMMU_LUT363L
  0x00039F90, //GFXMMU_LUT363H
  0x00240C01, //GFXMMU_LUT364L
  0x0003A120, //GFXMMU_LUT364H
  0x00240C01, //GFXMMU_LUT365L
  0x0003A2B0, //GFXMMU_LUT365H
  0x00240C01, //GFXMMU_LUT366L
  0x0003A440, //GFXMMU_LUT366H
  0x00230C01, //GFXMMU_LUT367L
  0x0003A5D0, //GFXMMU_LUT367H
  0x00230D01, //GFXMMU_LUT368L
  0x0003A740, //GFXMMU_LUT368H
  0x00230D01, //GFXMMU_LUT369L
  0x0003A8B0, //GFXMMU_LUT369H
  0x00230D01, //GFXMMU_LUT370L
  0x0003AA20, //GFXMMU_LUT370H
  0x00220D01, //GFXMMU_LUT371L
  0x0003AB90, //GFXMMU_LUT371H
  0x00220E01, //GFXMMU_LUT372L
  0x0003ACE0, //GFXMMU_LUT372H
  0x00220E01, //GFXMMU_LUT373L
  0x0003AE30, //GFXMMU_LUT373H
  0x00220E01, //GFXMMU_LUT374L
  0x0003AF80, //GFXMMU_LUT374H
  0x00210E01, //GFXMMU_LUT375L
  0x0003B0D0, //GFXMMU_LUT375H
  0x00210F01, //GFXMMU_LUT376L
  0x0003B200, //GFXMMU_LUT376H
  0x00210F01, //GFXMMU_LUT377L
  0x0003B330, //GFXMMU_LUT377H
  0x00200F01, //GFXMMU_LUT378L
  0x0003B460, //GFXMMU_LUT378H
  0x00201001, //GFXMMU_LUT379L
  0x0003B570, //GFXMMU_LUT379H
  0x00201001, //GFXMMU_LUT380L
  0x0003B680, //GFXMMU_LUT380H
  0x001F1101, //GFXMMU_LUT381L
  0x0003B780, //GFXMMU_LUT381H
  0x001F1101, //GFXMMU_LUT382L
  0x0003B870, //GFXMMU_LUT382H
  0x001E1101, //GFXMMU_LUT383L
  0x0003B960, //GFXMMU_LUT383H
  0x001E1201, //GFXMMU_LUT384L
  0x0003BA30, //GFXMMU_LUT384H
  0x001D1201, //GFXMMU_LUT385L
  0x0003BB00, //GFXMMU_LUT385H
  0x001D1301, //GFXMMU_LUT386L
  0x0003BBB0, //GFXMMU_LUT386H
  0x001C1401, //GFXMMU_LUT387L
  0x0003BC50, //GFXMMU_LUT387H
  0x001B1401, //GFXMMU_LUT388L
  0x0003BCE0, //GFXMMU_LUT388H
  0x001A1501, //GFXMMU_LUT389L
  0x0003BD50 //GFXMMU_LUT389H
};

#ifdef __cplusplus
}
#endif
#endif /*__ gfxmmu_lut_H */

/**
  * @}
  */

/**
  * @}
  */
