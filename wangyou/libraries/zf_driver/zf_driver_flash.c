/*********************************************************************************************************************
* CYT2BL3 Opensourec Library ���� CYT2BL3 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� CYT2BL3 ��Դ���һ����
*
* CYT2BL3 ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          zf_driver_flash
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 9.40.1
* ����ƽ̨          CYT2BL3
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2024-1-8       pudding            first version
********************************************************************************************************************/


#include "zf_common_debug.h"
#include "zf_driver_flash.h"


flash_data_union flash_union_buffer[FLASH_PAGE_LENGTH];                    // FLASH ���������ݻ�����
static vuint8 flash_init_flag = 0;

//-------------------------------------------------------------------------------------------------------------------
//  �������      У��FLASHҳ�Ƿ�������
//  ����˵��      sector_num    ������д0  �˴�������Ų���ʵ�����ã�ֻ�������ӿ�
//  ����˵��      page_num      ��Ҫд���ҳ���   ������Χ <0 - 47>
//  ���ز���      ����1�����ݣ�����0û�����ݣ������Ҫ�������ݵ�����д���µ�������Ӧ�ö������������в�������
//  ʹ��ʾ��      flash_check(0, 0); 	// У��0ҳ�Ƿ�������
//  ��ע��Ϣ			
//-------------------------------------------------------------------------------------------------------------------
uint8 flash_check (uint32 sector_num, uint32 page_num)
{
    zf_assert(FLASH_PAGE_NUM > page_num);
    zf_assert(flash_init_flag);				// �û�δ��ʼ��flash����Ա���

    return !Cy_WorkFlashBlankCheck((page_num * FLASH_PAGE_SIZE + FLASH_BASE_ADDR), CY_FLASH_DRIVER_BLOCKING);
}

//-------------------------------------------------------------------------------------------------------------------
//  �������      ����ҳ
//  ����˵��      sector_num    ������д0  �˴�������Ų���ʵ�����ã�ֻ�������ӿ�
//  ����˵��      page_num      ��Ҫд���ҳ���   ������Χ <0 - 47>
//  ���ز���      void
//  ʹ��ʾ��      flash_erase_page(0, 0);
//  ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void flash_erase_page (uint32 sector_num, uint32 page_num)
{
    zf_assert(FLASH_PAGE_NUM > page_num);
    zf_assert(flash_init_flag);				// �û�δ��ʼ��flash����Ա���

    Cy_FlashSectorErase((page_num * FLASH_PAGE_SIZE + FLASH_BASE_ADDR), CY_FLASH_DRIVER_BLOCKING);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡһҳ
// ����˵��     sector_num      ������д0  �˴�������Ų���ʵ�����ã�ֻ�������ӿ�
// ����˵��     page_num        ��ǰ����ҳ�ı��   ������Χ <0 - 47>
// ����˵��     buf             ��Ҫ��ȡ�����ݵ�ַ   ������������ͱ���Ϊuint32
// ����˵��     len             ��Ҫд������ݳ���   ������Χ 1 - 511
// ���ز���     void
// ʹ��ʾ��     flash_read_page(0, 0, data_buffer, 256);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void flash_read_page(uint32 sector_num, uint32 page_num, uint32 *buf, uint32 len)
{
    uint32 data_cont = 0;
    zf_assert(FLASH_PAGE_NUM > page_num);
    zf_assert(FLASH_PAGE_LENGTH >= len);
    zf_assert(flash_init_flag);				// �û�δ��ʼ��flash����Ա���

    uint32 * flash_addr = (uint32 *)FLASH_BASE_ADDR;
    flash_addr = flash_addr + (page_num * FLASH_PAGE_SIZE / FLASH_DATA_SIZE);
#if CY_CORE_CM7_0 || CY_CORE_CM7_1
    SCB_InvalidateDCache_by_Addr(flash_addr, len * FLASH_DATA_SIZE);
#endif
    for(data_cont = 0; data_cont < len; data_cont ++)
    {
        *buf ++ = *(flash_addr + (data_cont * FLASH_DATA_SIZE));
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  �������      ���һҳ
//  ����˵��      sector_num      ������д0  �˴�������Ų���ʵ�����ã�ֻ�������ӿ�
//  ����˵��      page_num        ��ǰ����ҳ�ı��    ������Χ <0 - 47>
//  ����˵��      buf             ��Ҫд������ݵ�ַ   ������������ͱ���Ϊ uint32
//  ����˵��      len             ��Ҫд������ݳ���   ������Χ 1 - 511
//  ���ز���      void
//  ʹ��ʾ��      flash_write_page(0, 0, buf, 10);
//  ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void flash_write_page (uint32 sector_num, uint32 page_num, const uint32 *buf, uint32 len)
{
    zf_assert(FLASH_PAGE_NUM > page_num);
    zf_assert(FLASH_PAGE_LENGTH >= len);
    zf_assert(flash_init_flag);				// �û�δ��ʼ��flash����Ա���
    
    if(0 != flash_check(sector_num, page_num))
    {
        flash_erase_page(sector_num, page_num);
    }
    uint32 flash_addr = page_num * FLASH_PAGE_SIZE + FLASH_BASE_ADDR;
    
    for(int i = 0; i < len; i ++)
    {
      
        Cy_FlashWriteWork(flash_addr, buf, CY_FLASH_DRIVER_BLOCKING);
        flash_addr += 4;
        buf += 1;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ָ�� FLASH ��������ָ��ҳ���ȡ���ݵ�������
// ����˵��     sector_num      ������д0  �˴�������Ų���ʵ�����ã�ֻ�������ӿ�
// ����˵��     page_num        ��ǰ����ҳ�ı��   ������Χ <0 - 47>
// ���ز���     void
// ʹ��ʾ��     flash_read_page_to_buffer(0, 0, 1);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void flash_read_page_to_buffer (uint32 sector_num, uint32 page_num, uint32 len)
{
    uint32 data_cont = 0;
    zf_assert(FLASH_PAGE_NUM > page_num);
    zf_assert(FLASH_PAGE_LENGTH >= len);
    zf_assert(flash_init_flag);				// �û�δ��ʼ��flash����Ա���
	
    uint32 * flash_addr = (uint32 *)FLASH_BASE_ADDR;
    flash_addr = flash_addr + (page_num * FLASH_PAGE_SIZE / FLASH_DATA_SIZE);
#if CY_CORE_CM7_0 || CY_CORE_CM7_1
    SCB_InvalidateDCache_by_Addr(flash_addr, len * FLASH_DATA_SIZE);
#endif    
    for(data_cont = 0; data_cont < len; data_cont ++)
    {
        flash_union_buffer[data_cont].uint32_type = flash_addr[data_cont];
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ָ�� FLASH ��������ָ��ҳ��д�뻺����������
// ����˵��     sector_num      ������д0  �˴�������Ų���ʵ�����ã�ֻ�������ӿ�
// ����˵��     page_num        ��ǰ����ҳ�ı��   ������Χ <0 - 47>
// ���ز���     uint8           1-��ʾʧ�� 0-��ʾ�ɹ�
// ʹ��ʾ��     flash_write_page_from_buffer(0, 0, 1);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
uint8 flash_write_page_from_buffer (uint32 sector_num, uint32 page_num, uint32 len)
{
    zf_assert(FLASH_PAGE_NUM > page_num);
    zf_assert(FLASH_PAGE_LENGTH >= len);
    zf_assert(flash_init_flag);				// �û�δ��ʼ��flash����Ա���
	
    uint32 *data_pointer = (uint32 *)flash_union_buffer;

    flash_write_page(0, page_num, data_pointer, len);
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ������ݻ�����
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     flash_buffer_clear();
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void flash_buffer_clear (void)
{
    memset(flash_union_buffer, 0xFF, FLASH_PAGE_LENGTH * sizeof(flash_data_union));
}

//-------------------------------------------------------------------------------------------------------------------
// �������     flash��ʼ��
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     flash_init();
// ��ע��Ϣ     
//-------------------------------------------------------------------------------------------------------------------
void flash_init (void)
{
    Cy_FlashInit(false);
    flash_init_flag = 1;
}