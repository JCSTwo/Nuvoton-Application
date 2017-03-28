/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright (c) Nuvoton Technology Corp. All rights reserved.                                             */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#include "Driver\DrvUSB.h"
#include "MassStorage_ISP.h"

#define MASS_BUFFER_SIZE        48              /* Mass Storage command buffer size */
#define STORAGE_BUFFER_SIZE     512             /* Data transfer buffer size in 512 bytes alignment */
#define UDC_SECTOR_SIZE         512             /* logic sector size */

#define VENDOR_ID               0x0416
#define PRODUCT_ID              0x1001

int32_t  g_dataFlashTotalSectors = 0;
uint32_t g_u32FlashSize;
uint8_t  g_u8SecurityLockBit = 0x2;

uint8_t volatile MassLUN = 0;
uint8_t volatile F_DATA_FLASH_LUN = 0xFF;

/* DATA Flash Programming API */
//extern void DataFlashRead(uint32_t addr, uint32_t buffer);
extern void FMC_ReadPage(uint32_t u32startAddr, uint32_t *u32buff);
extern void DataFlashWrite(uint32_t addr, uint32_t buffer);

extern uint8_t u8FormatData[62];

/* USB flow control variables */
uint8_t g_u8UsbState;
uint8_t g_u8Address;
uint8_t g_u8Config;
uint8_t g_u8Flag;
uint8_t g_u8BulkState;

uint8_t g_au8SenseKey[4];

uint32_t g_u32DataFlashStartAddr;
uint32_t g_u32Address;
uint32_t g_u32Length;
uint32_t g_u32LbaAddress;
uint32_t g_u32BytesInStorageBuf;
uint32_t g_u32OutToggle;    // for Error Detection
uint8_t g_u8Size;
int32_t g_bCBWInvalid;
int32_t udcOnLine = 0;

/* CBW/CSW variables */
struct CBW g_sCBW;
struct CSW g_sCSW;

uint32_t MassBlock[MASS_BUFFER_SIZE / 4];
uint32_t Storage_Block[STORAGE_BUFFER_SIZE / 4];

#define MassCMD_BUF        ((uint32_t)&MassBlock[0])
#define STORAGE_DATA_BUF   ((uint32_t)&Storage_Block[0])

// code = 5Ah, Mode Sense
static uint8_t ModePage_01[12] = {
    0x01, 0x0A, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,
    0x03, 0x00, 0x00, 0x00
};

static uint8_t ModePage_05[32] = {
    0x05, 0x1E, 0x13, 0x88, 0x08, 0x20, 0x02, 0x00,
    0x01, 0xF4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x05, 0x1E, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x68, 0x00, 0x00
};

static uint8_t ModePage_1B[12] = {
    0x1B, 0x0A, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
};

static uint8_t ModePage_1C[8] = {
    0x1C, 0x06, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00
};

//======================================================
const uint8_t MassDeviceDescriptor[LEN_DEVICE] = {
    LEN_DEVICE,         // bLength
    DESC_DEVICE,        // bDescriptorType
    0x10, 0x01,         // bcdUSB
    0x00,               // bDeviceClass
    0x00,               // bDeviceSubClass
    0x00,               // bDeviceProtocol
    MAX_PACKET_SIZE0,   // bMaxPacketSize0
    (VENDOR_ID & 0xFF),
    (VENDOR_ID >> 8),   // idVendor
    (PRODUCT_ID & 0xFF),
    (PRODUCT_ID >> 8),  // idProduct
    0x00, 0x00,         // bcdDevice
    0x01,               // iManufacture
    0x02,               // iProduct
    0x01,               // iSerialNumber
    0x01                // bNumConfigurations
};

const uint8_t MassConfigurationBlock[LEN_CONFIG + LEN_INTERFACE + LEN_ENDPOINT * 2] = {
    LEN_CONFIG,                                             // bLength
    DESC_CONFIG,                                            // bDescriptorType
    (LEN_CONFIG + LEN_INTERFACE + LEN_ENDPOINT * 2), 0x00,  // wTotalLength
    0x01,                                                   // bNumInterfaces
    0x01,                                                   // bConfigurationValue
    0x00,                                                   // iConfiguration
    0xC0,                                                   // bmAttributes
    0x32,                                                   // MaxPower

    /* const BYTE cbyInterfaceDescriptor[LEN_INTERFACE] = */
    LEN_INTERFACE,  // bLength
    DESC_INTERFACE, // bDescriptorType
    0x00,           // bInterfaceNumber
    0x00,           // bAlternateSetting
    0x02,           // bNumEndpoints
    0x08,           // bInterfaceClass
    0x05,           // bInterfaceSubClass
    0x50,           // bInterfaceProtocol
    0x00,           // iInterface

    /* const BYTE cbyEndpointDescriptor1[LEN_ENDPOINT] = */
    LEN_ENDPOINT,           // bLength
    DESC_ENDPOINT,          // bDescriptorType
    0x82,                   // bEndpointAddress
    EP_BULK,                // bmAttributes
    MAX_PACKET_SIZE, 0x00,  // wMaxPacketSize
    0x00,                   // bInterval

    /* const BYTE cbyEndpointDescriptor2[LEN_ENDPOINT] = */
    LEN_ENDPOINT,           // bLength
    DESC_ENDPOINT,          // bDescriptorType
    0x03,                   // bEndpointAddress
    EP_BULK,                // bmAttributes
    MAX_PACKET_SIZE, 0x00,  // wMaxPacketSize
    0x00                    // bInterval
};

const uint8_t MassStringDescriptor0[4] = {
    4,              // bLength
    DESC_STRING,    // bDescriptorType
    0x09, 0x04
};

const uint8_t MassStringDescriptor1[16] = {
    16,             // bLength
    DESC_STRING,    // bDescriptorType
    'N', 0, 'u', 0, 'v', 0, 'o', 0, 't', 0, 'o', 0, 'n', 0
};

const uint8_t InquiryID[36] = {
    0x00,                   // Peripheral Device Type
    0x80,                   // RMB
    0x00,                   // ISO/ECMA, ANSI Version
    0x00,                   // Response Data Format
    0x1F, 0x00, 0x00, 0x00, // Additional Length

    /* Vendor Identification */
    'N', 'u', 'v', 'o', 't', 'o', 'n', ' ',

    /* Product Identification */
    '1', '0', '0', ' ', 'M', 'a', 's', 's', ' ', 'S', 't', 'o', 'r', 'a', 'g', 'e',

    /* Product Revision */
    '1', '.', '0', '0'
};


static __INLINE uint32_t get_be32(uint8_t *buf)
{
    return ((uint32_t) buf[0] << 24) | ((uint32_t) buf[1] << 16) |
           ((uint32_t) buf[2] <<  8) | ((uint32_t) buf[3]);
}

void RoughDelay(uint32_t t)
{
    volatile int32_t delay;
    delay = t;

    while (delay-- >= 0);
}

void my_memcpy(void *dest, void *src, int32_t size)
{
    int32_t i;

    for (i = 0; i < size; i++) {
        *((uint8_t *)dest + i) = *((uint8_t *)src + i);
    }
}


void ModeSenseCommand(void)
{
    uint8_t i, j;
    uint8_t NumHead, NumSector;
    uint16_t NumCyl = 0;
    /* Clear the command buffer */
    *((uint32_t *)MassCMD_BUF) = 0;
    *((uint32_t *)MassCMD_BUF + 1) = 0;

    switch (g_sCBW.au8Data[0]) {
        case 0x1C: {
            outpb(MassCMD_BUF, 15);
            i = 8;

            for (j = 0; j < 8; j++, i++) {
                outpb(MassCMD_BUF + i, ModePage_1C[j]);
            }

            break;
        }

        case 0x3F: {
            outpb(MassCMD_BUF, 0x47);
            i = 8;

            for (j = 0; j < 12; j++, i++) {
                outpb(MassCMD_BUF + i, ModePage_01[j]);
            }

            for (j = 0; j < 32; j++, i++) {
                outpb(MassCMD_BUF + i, ModePage_05[j]);
            }

            for (j = 0; j < 12; j++, i++) {
                outpb(MassCMD_BUF + i, ModePage_1B[j]);
            }

            for (j = 0; j < 8; j++, i++) {
                outpb(MassCMD_BUF + i, ModePage_1C[j]);
            }

            NumHead = 2;
            NumSector = 64;

            if (g_sCBW.bCBWLUN == F_DATA_FLASH_LUN) {
                NumCyl = g_dataFlashTotalSectors / 128;
            }

            outpb(MassCMD_BUF + 24, NumHead);
            outpb(MassCMD_BUF + 25, NumSector);
            outpb(MassCMD_BUF + 28, (uint8_t)(NumCyl >> 8));
            outpb(MassCMD_BUF + 29, (uint8_t)(NumCyl & 0x00ff));
            break;
        }

        default:
            g_au8SenseKey[0] = 0x05;
            g_au8SenseKey[1] = 0x24;
            g_au8SenseKey[2] = 0x00;
    }
}


void RdFmtCapCommand(void)
{
    uint32_t i;
    uint32_t TotalSectors = 0;

    for (i = 0; i < 36; i++) {
        outpb(MassCMD_BUF + i, 0);
    }

    outpb(MassCMD_BUF + 3, 0x10);

    if (g_sCBW.bCBWLUN == F_DATA_FLASH_LUN) {
        TotalSectors = g_dataFlashTotalSectors;
    }

    outpb(MassCMD_BUF + 4, *((uint8_t *)&TotalSectors + 3));
    outpb(MassCMD_BUF + 5, *((uint8_t *)&TotalSectors + 2));
    outpb(MassCMD_BUF + 6, *((uint8_t *)&TotalSectors + 1));
    outpb(MassCMD_BUF + 7, *((uint8_t *)&TotalSectors + 0));
    outpb(MassCMD_BUF + 8, 0x02);
    outpb(MassCMD_BUF + 10, 0x02);
    outpb(MassCMD_BUF + 12, *((uint8_t *)&TotalSectors + 3));
    outpb(MassCMD_BUF + 13, *((uint8_t *)&TotalSectors + 2));
    outpb(MassCMD_BUF + 14, *((uint8_t *)&TotalSectors + 3));
    outpb(MassCMD_BUF + 15, *((uint8_t *)&TotalSectors + 0));
    outpb(MassCMD_BUF + 18, 0x02);
}


void RdCurCapCommand(void)
{
    int i;
    uint32_t temp;

    for (i = 0; i < 36; i++) {
        outpb(MassCMD_BUF + i, 0);
    }

    if (g_sCBW.bCBWLUN == F_DATA_FLASH_LUN) {
        temp = g_dataFlashTotalSectors - 1;
    }

    outpb(MassCMD_BUF + 0, *((uint8_t *)&temp + 3));
    outpb(MassCMD_BUF + 1, *((uint8_t *)&temp + 2));
    outpb(MassCMD_BUF + 2, *((uint8_t *)&temp + 1));
    outpb(MassCMD_BUF + 3, *((uint8_t *)&temp + 0));
    outpb(MassCMD_BUF + 6, 0x02);
}


//======================================================
// USB Configuration
//======================================================
void UsbCfg(void)
{
    /* Init the USB device address to 0x0 */
    outp32(&USBD->FADDR, 0x00);
    /* Init control end point */
    USBD->STBUFSEG = 0x1F << USBD_STBUFSEG_STBUFSEG_Pos;    /* Buffer for setup packet */
    _DRVUSB_SET_CFG_EP0(EPT_CFGP);                          /* Clear EP0 CTRL IN Ready Flag */
    _DRVUSB_SET_CFG(0, CFG0_SETTING);                       /* EP0 ==> control IN end point, address 0 */
    _DRVUSB_SET_EP_BUF(0, USB_SRAM_BASE + BUF0_SETTING);    /* Buffer for EP0 */
    _DRVUSB_SET_CFG_EP1(EPT_CFGP);                          /* Clear EP1 CTRL OUT Ready Flag */
    _DRVUSB_SET_CFG(1, CFG1_SETTING);                       /* EP1 ==> control OUT end point, address 0 */
    _DRVUSB_SET_EP_BUF(1, USB_SRAM_BASE + BUF1_SETTING);    /* Buffer for EP1 */
    /* Init Bulk IN end point */
    _DRVUSB_SET_CFG_EP2(EPT_CFGP);                          /* Clear EP2 BULK IN Ready Flag */
    _DRVUSB_SET_CFG(2, CFG2_SETTING);                       /* EP2 ==> Bulk IN end point, address 2 */
    _DRVUSB_SET_EP_BUF(2, USB_SRAM_BASE + BUF_BULK1);       /* Buffer for EP2 (Bulk IN) */
    /* Init Bulk OUT end point */
    _DRVUSB_SET_CFG_EP3(EPT_CFGP);                          /* Clear EP3 BULK OUT Ready Flag */
    _DRVUSB_SET_CFG(3, CFG3_SETTING);                       /* EP3 ==> Bulk OUT end point, address 3 */
    _DRVUSB_SET_EP_BUF(3, USB_SRAM_BASE + BUF_BULK0);       /* Buffer for EP3 (Bulk OUT) */
    g_u8Address = 0;    /* Device address */
    g_u8Config = 0;
    g_u8Flag = 0;       /* Flag to record the USB request */
    g_u8BulkState = BULK_CBW;
    g_au8SenseKey[0] = 0;
    g_au8SenseKey[1] = 0;
    g_au8SenseKey[2] = 0;
    g_bCBWInvalid = FALSE;
    g_u32Address = 0;
    g_u32Length = 0;
    g_u8Size = 0;
    g_sCSW.dCSWSignature = CSW_SIGNATURE;
    udcOnLine = 0;
    g_u32OutToggle = 0; // For Error Detection
}

//======================================================
// USB Floating Detect
//======================================================
void UsbFdt(void)
{
    _DRVUSB_SET_EVENT_FLAG(INTSTS_FLDET);

    if (USBD->FLDET & USBD_FLDET_FLDET_Msk) {
        /* USB Plug In */
        if (g_u8UsbState & USB_STATE_ATTACHED) {
            /* Do nothing if it is already attached */
            return;
        }

        /* Update USB state */
        g_u8UsbState = USB_STATE_ATTACHED;
        /* Init the end points */
        UsbCfg();
        /* Enable USB and enable PHY */
        _DRVUSB_ENABLE_USB();
    } else {
        /* USB Un-plug */
        /* Update USB state */
        g_u8UsbState = USB_STATE_DETACHED;
        udcOnLine = 0;
        /* Disable USB, Disable remote weak up, and disable PHY */
        _DRVUSB_DISABLE_USB();
    }
}


//======================================================
// USB Bus Event
//======================================================
void UsbBus(void)
{
    uint8_t u8ATTR = *((__IO uint32_t *)(&USBD->ATTR)) & 0xFF;
    uint8_t u8UsbState;
    /* Clear event flag */
    _DRVUSB_SET_EVENT_FLAG(INTSTS_BUS);
    u8UsbState = g_u8UsbState;

    /* Just return if USB detached. */
    if (u8UsbState == USB_STATE_DETACHED) {
        return;
    }

    /* Process USB bus command if USB is attached. */
    if (u8ATTR & ATTR_USBRST) {
        /* Enable USB and enable PHY */
        _DRVUSB_ENABLE_USB();
        /* Enter default state */
        u8UsbState = USB_STATE_DEFAULT;
        /* Configure the end-points */
        UsbCfg();
    } else if (u8ATTR & ATTR_SUSPEND) {
        /* Enable USB but disable PHY */
        _DRVUSB_DISABLE_PHY();
        /* Update the USB state */
        u8UsbState |= USB_STATE_FLAG_SUSPENDED;
    } else if (u8ATTR & ATTR_RESUME) {
        /* Enable USB and enable PHY */
        _DRVUSB_ENABLE_USB();
        /* Clear the suspend state */
        u8UsbState &= ~USB_STATE_FLAG_SUSPENDED;
    }

    g_u8UsbState = u8UsbState;
}


//======================================================
int32_t UsbStdReq(void)
{
    int volatile len, idx;
    uint8_t tmp[8];
    my_memcpy(tmp, g_au8UsbSetup, 8);

    /* check if after DEFAULT state (RESET) */
    if ((g_u8UsbState & USB_STATE_FLAG_DEFAULT) == 0x00) {
        return FALSE;
    }

    /* parse the request in setup packet */
    switch (tmp[1]) {
        case GET_DESCRIPTOR: {
            len = tmp[6] + ((uint32_t)tmp[7] << 8);

            switch (tmp[3]) {
                // Get Device Descriptor
                case DESC_DEVICE: {
                    udcOnLine = 1;

                    if (len > LEN_DEVICE) {
                        len = LEN_DEVICE;
                    }

                    my_memcpy(g_au8UsbCtrl, (void *)MassDeviceDescriptor, len);
                    break;
                }

                // Get Configuration Descriptor
                case DESC_CONFIG: {
                    if (len > MassConfigurationBlock[2]) {
                        len = MassConfigurationBlock[2];
                    }

                    my_memcpy(g_au8UsbCtrl, (void *)MassConfigurationBlock, len);
                    break;
                }

                // Get String Descriptor
                case DESC_STRING: {
                    // Get Language
                    if (tmp[4] == 0) {
                        if (len > MassStringDescriptor0[0]) {
                            len = MassStringDescriptor0[0];
                        }

                        my_memcpy(g_au8UsbCtrl, (void *)MassStringDescriptor0, len);
                        break;
                    }

                    // Get String Descriptor
                    if (len > MassStringDescriptor1[0]) {
                        len = MassStringDescriptor1[0];
                    }

                    my_memcpy(g_au8UsbCtrl, (void *)MassStringDescriptor1, len);
                    break;
                }

                default:
                    return FALSE;
            }

            _DRVUSB_SET_CFG(0, DATA1(CFG0_SETTING));
            _DRVUSB_TRIG_EP(0, len);
            g_u8Flag = FLAG_OUT_ACK;
            return TRUE;
        }

        case SET_ADDRESS: {
            /* Accept set address command only before USB configured */
            if ((g_u8UsbState & USB_STATE_FLAG_CONFIGURED) == 0) {
                /* Get the device address */
                g_u8Address = tmp[2];
                _DRVUSB_SET_CFG(0, DATA1(CFG0_SETTING));
                _DRVUSB_TRIG_EP(0, 0);
                g_u8Flag = FLAG_SET_ADDRESS;
                return TRUE;
            }

            return FALSE;
        }

        case SET_CONFIGURATION: {
            g_u8Config = tmp[2];
            _DRVUSB_SET_CFG(0, DATA1(CFG0_SETTING));
            _DRVUSB_TRIG_EP(0, 0);
            g_u8Flag = FLAG_SET_CONFIG;
            return TRUE;
        }

        case CLEAR_FEATURE:
        case SET_FEATURE: {
            if (g_bCBWInvalid) {
                /* Invalid CBW */
            } else if ((tmp[0] == 2) && (tmp[2] == FEATURE_ENDPOINT_HALT)) {
                uint32_t u32CfgAddr;
                uint8_t  u8Cfg;
                u32CfgAddr = ((tmp[4] & 0xF) << 4) + USBD_CFGP0;
                u8Cfg = *((__IO uint32_t *)(u32CfgAddr)) & 0xFF;

                if (tmp[1] == CLEAR_FEATURE) {
                    u8Cfg &= ~EPT_stall;
                } else {
                    u8Cfg |= EPT_stall;
                }

                *((__IO uint32_t *)(u32CfgAddr)) = u8Cfg;
            } else {
                return FALSE;
            }

            _DRVUSB_SET_CFG(0, DATA1(CFG0_SETTING));
            _DRVUSB_TRIG_EP(0, 0);
            return TRUE;
        }

        case GET_CONFIGURATION: {
            g_au8UsbCtrl[0] = g_u8Config;
            _DRVUSB_SET_CFG(0, DATA1(CFG0_SETTING));
            _DRVUSB_TRIG_EP(0, 1);
            g_u8Flag = FLAG_OUT_ACK;
            return TRUE;
        }

        case GET_STATUS: {
            uint8_t u8Data;
            u8Data = tmp[0];

            // Device
            if (u8Data == 0x80) {
                g_au8UsbCtrl[0] = 1;    // Self-Powered
            }
            // Interface
            else if (u8Data == 0x81) {
                g_au8UsbCtrl[0] = 0;
            }
            // Endpoint
            else if (u8Data == 0x82) {
                uint32_t u32CfgAddr;
                u8Data = tmp[4];
                u32CfgAddr = ((u8Data & 0xF) << 4) + USBD_CFGP0;
                g_au8UsbCtrl[0] = (inp32(u32CfgAddr) >> 1) & 1;
            } else {
                return FALSE;
            }

            g_au8UsbCtrl[1] = 0;
            _DRVUSB_SET_CFG(0, DATA1(CFG0_SETTING));
            _DRVUSB_TRIG_EP(0, 1);
            g_u8Flag = FLAG_OUT_ACK;
            return TRUE;
        }

        case GET_INTERFACE: {
            g_au8UsbCtrl[0] = 0;
            _DRVUSB_SET_CFG(0, DATA1(CFG0_SETTING));
            _DRVUSB_TRIG_EP(0, 1);
            g_u8Flag = FLAG_OUT_ACK;
            return TRUE;
        }

        case SET_INTERFACE: {
            g_u8Flag = FLAG_SET_INTERFACE;
            _DRVUSB_SET_CFG(0, DATA1(CFG0_SETTING));
            _DRVUSB_TRIG_EP(0, 0);
            return TRUE;
        }

        default:
            return FALSE;
    }
}


//======================================================
int32_t UsbClassReq(void)
{
    uint8_t tmp[8];
    uint16_t u16sum = 0;
    my_memcpy(tmp, g_au8UsbSetup, 8);
    u16sum = tmp[2] + tmp[3] + tmp[4] + tmp[5];

    if (u16sum != 0) {
        return FALSE;
    }

    switch (tmp[1]) {
        // Bulk-Only Mass Storage Reset
        case BULK_ONLY_MASS_STORAGE_RESET: {
            u16sum = tmp[6] + tmp[7];

            if (u16sum != 0) {
                return FALSE;
            }

            g_bCBWInvalid = FALSE;
            _DRVUSB_SET_CFG(0, DATA1(CFG0_SETTING));
            _DRVUSB_TRIG_EP(0, 0);
            _DRVUSB_SET_CFG(2, DATA0(CFG2_SETTING));
            g_u8BulkState = BULK_CBW;
            _DRVUSB_TRIG_EP(3, 31);
            return TRUE;
        }

        // Get Max LUN
        case GET_MAX_LUN: {
            if ((tmp[6] != 1) || (tmp[7] != 0)) {
                return FALSE;
            }

            g_au8UsbCtrl[0] = MassLUN - 1;
            _DRVUSB_SET_CFG(0, DATA1(CFG0_SETTING));
            _DRVUSB_SET_CFG(3, DATA0(CFG3_SETTING));
            _DRVUSB_TRIG_EP(0, 1);
            g_u8Flag = FLAG_OUT_ACK;
            return TRUE;
        }
    }

    return FALSE;
}


//======================================================
int32_t UsbSetup(void)
{
    /* Clear the data IN/OUT ready flag of control end-points */
    _DRVUSB_SET_CFG_EP0(CFGP_CLRRDY);
    _DRVUSB_SET_CFG_EP1(CFGP_CLRRDY);

    /* USB device request in setup packet: offset 0, D[6..5]: 0=Stardard, 1=Class, 2=Vendor, 3=Reserved */
    switch (g_au8UsbSetup[0] & 0x60) {
        case REQ_STANDARD:
            return UsbStdReq();

        case REQ_CLASS:
            return UsbClassReq();

        default:
            return FALSE;
    }
}


//======================================================
void UsbInAck(void)
{
    switch (g_u8Flag) {
        // Out ACK
        case FLAG_OUT_ACK:
            _DRVUSB_TRIG_EP(1, 0);
            break;

        // Set Address
        case FLAG_SET_ADDRESS: {
            if (g_u8Address == 0) {
                g_u8UsbState = USB_STATE_DEFAULT;
            } else {
                g_u8UsbState = USB_STATE_ADDRESS;
            }

            outp32(&USBD->FADDR, g_u8Address);
            break;
        }

        // Set Interface
        case FLAG_SET_INTERFACE:

        // Set Configuration
        case FLAG_SET_CONFIG: {
            if (g_u8Config == 0) {
                g_u8UsbState = USB_STATE_ADDRESS;
            } else {
                g_u8UsbState = USB_STATE_CONFIGURED;
                // ---------------------------------
                _DRVUSB_SET_CFG(2, DATA0(CFG2_SETTING));
                _DRVUSB_TRIG_EP(3, 31);
                // ---------------------------------
            }

            break;
        }

        // Get Max. LUN
        case FLAG_GET_MAX_LUN:
            _DRVUSB_TRIG_EP(1, 0);
            return;

        default:
            return;
    }

    g_u8Flag = 0;
}

//======================================================
void UsbOutAck(void)
{
    switch (g_u8Flag) {
        // Get Max. LUN
        case FLAG_GET_MAX_LUN: {
            // ---------------------------------
            _DRVUSB_SET_CFG(2, DATA0(CFG2_SETTING));
            _DRVUSB_TRIG_EP(3, 31);
            // ---------------------------------
            break;
        }

        default:
            return;
    }

    g_u8Flag = 0;
}


//======================================================
void UsbRead(void)
{
    uint32_t u32Len, u32Length, u32Address;
    uint8_t u8Size;

    /* DATA0/DATA1 Toggle */
    if (*((__IO uint32_t *)(&USBD->EP[2].BUFSEG)) == BUF_BULK1) {
        _DRVUSB_SET_EP_BUF(2, USB_SRAM_BASE + BUF_BULK0);
    } else {
        _DRVUSB_SET_EP_BUF(2, USB_SRAM_BASE + BUF_BULK1);
    }

    u8Size = g_u8Size;
    u32Length = g_u32Length;
    u32Address = g_u32Address;
    /* Trigger to send out the data packet */
    _DRVUSB_TRIG_EP(2, u8Size);
    u32Length -= u8Size;
    g_u32BytesInStorageBuf -= u8Size;

    if (u32Length) {
        if (g_u32BytesInStorageBuf) {
            /* Prepare next data packet */
            u8Size = MAX_PACKET_SIZE;

            if (u8Size > u32Length) {
                u8Size = u32Length;
            }

            if (*((__IO uint32_t *)(&USBD->EP[2].BUFSEG)) == BUF_BULK1) {
                my_memcpy(g_au8UsbBulk0, (int8_t *)u32Address, u8Size);
            } else {
                my_memcpy(g_au8UsbBulk1, (int8_t *)u32Address, u8Size);
            }

            u32Address += u8Size;
        } else {
            u32Len = u32Length;

            if (u32Len > STORAGE_BUFFER_SIZE) {
                u32Len = STORAGE_BUFFER_SIZE;
            }

            //DataFlashRead(g_u32LbaAddress, (uint32_t)STORAGE_DATA_BUF);
            FMC_ReadPage(g_u32LbaAddress, Storage_Block);
            g_u32BytesInStorageBuf = u32Len;
            g_u32LbaAddress += u32Len;
            u32Address = STORAGE_DATA_BUF;
            /* Prepare next data packet */
            u8Size = MAX_PACKET_SIZE;

            if (u8Size > u32Length) {
                u8Size = u32Length;
            }

            if (*((__IO uint32_t *)(&USBD->EP[2].BUFSEG)) == BUF_BULK1) {
                my_memcpy(g_au8UsbBulk0, (int8_t *)u32Address, u8Size);
            } else {
                my_memcpy(g_au8UsbBulk1, (int8_t *)u32Address, u8Size);
            }

            u32Address += u8Size;
        }

        g_u8Size = u8Size;
        g_u32Address = u32Address;
    }

    g_u32Length = u32Length;
}


void UsbBulkInAck(void)
{
    int32_t volatile idx;

    if (g_u8BulkState == BULK_CSW) {
        /* Prepare to receive the CBW */
        g_u8BulkState = BULK_CBW;
        _DRVUSB_SET_EP_BUF(3, USB_SRAM_BASE + BUF_BULK0);
        _DRVUSB_TRIG_EP(3, 31);
    } else if (g_u8BulkState == BULK_IN) {
        switch (g_sCBW.u8OPCode) {
            case UFI_READ_FORMAT_CAPACITY:
            case UFI_READ_CAPACITY:
            case UFI_MODE_SENSE_10:
            case UFI_READ_10: {
                if (g_u32Length > 0) {
                    UsbRead();
                    return;
                }
            }

            case UFI_INQUIRY: {
                g_sCSW.dCSWDataResidue = 0;
                g_sCSW.bCSWStatus = 0;
                break;
            }

            case UFI_VERIFY_10:
            case UFI_WRITE_10: {
                int32_t tmp;
                tmp = g_sCBW.dCBWDataTransferLength - STORAGE_BUFFER_SIZE;

                if (tmp < 0) {
                    tmp = 0;
                }

                g_sCSW.dCSWDataResidue = tmp;
                g_sCSW.bCSWStatus = 0;
                break;
            }

            case UFI_TEST_UNIT_READY: {
                g_sCSW.dCSWDataResidue = 0;
                g_sCSW.bCSWStatus = 0;
                break;
            }

            default: {
                /* Unsupported commmand. Return command fail status */
                g_sCSW.dCSWDataResidue = g_sCBW.dCBWDataTransferLength;
                g_sCSW.bCSWStatus = 0x01;
                break;
            }
        }

        /* Return the CSW */
        _DRVUSB_SET_EP_BUF(2, USB_SRAM_BASE + BUF_BULK1);
        my_memcpy(g_au8UsbBulk1, &g_sCSW.dCSWSignature, 16);
        g_u8BulkState = BULK_CSW;
        _DRVUSB_TRIG_EP(2, 13);
    }
}

//======================================================
void UsbWrite(void)
{
    uint32_t lba, len;
    uint32_t u32Address, u32Length;
    u32Address = g_u32Address;
    u32Length  = g_u32Length;

    if (u32Length > MAX_PACKET_SIZE) {
        if (*((__IO uint32_t *)(&USBD->EP[3].BUFSEG)) == BUF_BULK0) {
            _DRVUSB_SET_EP_BUF(3, USB_SRAM_BASE + BUF_BULK1);
            _DRVUSB_TRIG_EP(3, MAX_PACKET_SIZE);
            my_memcpy((char *)u32Address, g_au8UsbBulk0, MAX_PACKET_SIZE);
        } else {
            _DRVUSB_SET_EP_BUF(3, USB_SRAM_BASE + BUF_BULK0);
            _DRVUSB_TRIG_EP(3, MAX_PACKET_SIZE);
            my_memcpy((char *)u32Address, g_au8UsbBulk1, MAX_PACKET_SIZE);
        }

        u32Address += MAX_PACKET_SIZE;
        u32Length  -= MAX_PACKET_SIZE;

        /* Buffer full. Writer it to storage first. */
        if (u32Address >= (STORAGE_DATA_BUF + STORAGE_BUFFER_SIZE)) {
            if (g_sCBW.bCBWLUN == F_DATA_FLASH_LUN) {
                DataFlashWrite(g_u32DataFlashStartAddr, (uint32_t)STORAGE_DATA_BUF);
            }

            u32Address = STORAGE_DATA_BUF;
            g_u32DataFlashStartAddr += STORAGE_BUFFER_SIZE;
        }

        g_u32Address = u32Address;
        g_u32Length  = u32Length;
    } else {
        if (*((__IO uint32_t *)(&USBD->EP[3].BUFSEG)) == BUF_BULK0) {
            my_memcpy((char *)u32Address, g_au8UsbBulk0, u32Length);
        } else {
            my_memcpy((char *)u32Address, g_au8UsbBulk1, u32Length);
        }

        u32Address += u32Length;
        g_u32Address = u32Address;
        g_u32Length = 0;

        if (g_sCBW.u8OPCode == UFI_WRITE_10) {
            lba = get_be32(&g_sCBW.au8Data[0]);
            len = g_sCBW.dCBWDataTransferLength;

            if (g_sCBW.bCBWLUN == F_DATA_FLASH_LUN) {
                len = lba * UDC_SECTOR_SIZE + g_sCBW.dCBWDataTransferLength - g_u32DataFlashStartAddr;

                if (len) {
                    DataFlashWrite(g_u32DataFlashStartAddr, (uint32_t)STORAGE_DATA_BUF);
                }
            }
        }

        g_u8BulkState = BULK_IN;
        UsbBulkInAck();
    }
}


//======================================================
int32_t UsbBulkOutAck(void)
{
    uint8_t u8Len;
    int32_t i;

    if (g_u8BulkState == BULK_CBW) {
        u8Len = *((__IO uint32_t *)(&USBD->EP[3].MXPLD));

        /* Check Signature & length of CBW */
        if ((*(uint32_t *) g_au8UsbBulk0 != CBW_SIGNATURE) || (u8Len != 31)) {
            return FALSE;
        }

        /* Get the CBW */
        for (i = 0; i < u8Len; i++) {
            *((uint8_t *)(&g_sCBW.dCBWSignature) + i) = g_au8UsbBulk0[i];
        }

        /* Prepare to echo the tag from CBW to CSW */
        g_sCSW.dCSWTag = g_sCBW.dCBWTag;

        /* Parse Op-Code of CBW */
        switch (g_sCBW.u8OPCode) {
            case UFI_VERIFY_10:
            case UFI_TEST_UNIT_READY: {
                g_u8BulkState = BULK_IN;
                UsbBulkInAck();
                return TRUE;
            }

            case UFI_READ_FORMAT_CAPACITY: {
                if (g_u32Length == 0) {
                    g_u32Length = g_sCBW.dCBWDataTransferLength;
                    g_u32Address = MassCMD_BUF;
                }

                RdFmtCapCommand();
                g_u8BulkState = BULK_IN;

                if (g_u32Length > 0) {
                    if (g_u32Length > MAX_PACKET_SIZE) {
                        g_u8Size = MAX_PACKET_SIZE;
                    } else {
                        g_u8Size = g_u32Length;
                    }

                    my_memcpy(g_au8UsbBulk1, (char *)g_u32Address, g_u8Size);
                    g_u32Address += g_u8Size;
                    _DRVUSB_SET_EP_BUF(2, USB_SRAM_BASE + BUF_BULK0);
                    UsbRead();
                } else {
                    UsbBulkInAck();
                }

                return TRUE;
            }

            case UFI_READ_CAPACITY: {
                if (g_u32Length == 0) {
                    g_u32Length = g_sCBW.dCBWDataTransferLength;
                    g_u32Address = MassCMD_BUF;
                }

                RdCurCapCommand();
                g_u8BulkState = BULK_IN;

                if (g_u32Length > 0) {
                    if (g_u32Length > MAX_PACKET_SIZE) {
                        g_u8Size = MAX_PACKET_SIZE;
                    } else {
                        g_u8Size = g_u32Length;
                    }

                    my_memcpy(g_au8UsbBulk1, (char *)g_u32Address, g_u8Size);
                    g_u32Address += g_u8Size;
                    _DRVUSB_SET_EP_BUF(2, USB_SRAM_BASE + BUF_BULK0);
                    UsbRead();
                } else {
                    UsbBulkInAck();
                }

                return TRUE;
            }

            case UFI_MODE_SENSE_10: {
                if (g_u32Length == 0) {
                    g_u32Length = g_sCBW.dCBWDataTransferLength;
                    g_u32Address = MassCMD_BUF;
                }

                ModeSenseCommand();
                g_u8BulkState = BULK_IN;

                if (g_u32Length > 0) {
                    if (g_u32Length > MAX_PACKET_SIZE) {
                        g_u8Size = MAX_PACKET_SIZE;
                    } else {
                        g_u8Size = g_u32Length;
                    }

                    my_memcpy(g_au8UsbBulk1, (char *)g_u32Address, g_u8Size);
                    g_u32Address += g_u8Size;
                    _DRVUSB_SET_EP_BUF(2, USB_SRAM_BASE + BUF_BULK0);
                    UsbRead();
                } else {
                    UsbBulkInAck();
                }

                return TRUE;
            }

            case UFI_INQUIRY: {
                my_memcpy(g_au8UsbBulk1, (void *)InquiryID, 36);
                g_u8BulkState = BULK_IN;
                _DRVUSB_TRIG_EP(2, 36);
                return TRUE;
            }

            case UFI_READ_10: {
                /* Check if it is a new transfer */
                if (g_u32Length == 0) {
                    /* Prepare the data for Bulk IN transfer */
                    /* Get LBA address */
                    g_u32Address = get_be32(&g_sCBW.au8Data[0]);
                    g_u32LbaAddress = g_u32Address * UDC_SECTOR_SIZE;
                    g_u32Length = g_sCBW.dCBWDataTransferLength;
                    g_u32BytesInStorageBuf = g_u32Length;

                    if (g_sCBW.bCBWLUN == F_DATA_FLASH_LUN) {
                        i = g_u32Length;

                        if (i > STORAGE_BUFFER_SIZE) {
                            i = STORAGE_BUFFER_SIZE;
                        }

                        //DataFlashRead(g_u32Address * UDC_SECTOR_SIZE, (uint32_t)STORAGE_DATA_BUF);
                        FMC_ReadPage(g_u32Address * UDC_SECTOR_SIZE, Storage_Block);
                        g_u32BytesInStorageBuf = i;
                        g_u32LbaAddress += i;
                    }
                }

                g_u32Address = STORAGE_DATA_BUF;
                /* Indicate the next packet should be Bulk IN Data packet */
                g_u8BulkState = BULK_IN;

                if (g_u32BytesInStorageBuf > 0) {
                    /* Set the packet size */
                    if (g_u32BytesInStorageBuf > MAX_PACKET_SIZE) {
                        g_u8Size = MAX_PACKET_SIZE;
                    } else {
                        g_u8Size = g_u32BytesInStorageBuf;
                    }

                    /* Prepare the first data packet (DATA1) */
                    my_memcpy(g_au8UsbBulk1, (char *)g_u32Address, g_u8Size);
                    g_u32Address += g_u8Size;
                    /* kick - start */
                    _DRVUSB_SET_EP_BUF(2, USB_SRAM_BASE + BUF_BULK0);
                    UsbRead();
                } else {
                    UsbBulkInAck();
                }

                return TRUE;
            }

            case UFI_WRITE_10: {
                uint32_t u32Length;
                u32Length = g_u32Length;

                if (u32Length == 0) {
                    u32Length = g_sCBW.dCBWDataTransferLength;
                    g_u32Address = STORAGE_DATA_BUF;
                    g_u32DataFlashStartAddr = get_be32(&g_sCBW.au8Data[0]) * UDC_SECTOR_SIZE;
                }

                if (u32Length > 0) {
                    _DRVUSB_TRIG_EP(3, MAX_PACKET_SIZE);
                    g_u8BulkState = BULK_OUT;
                }

                g_u32Length = u32Length;
                return TRUE;
            }

            default: {
                /* Unsupported command */
                g_au8SenseKey[0] = 0x05;
                g_au8SenseKey[1] = 0x20;
                g_au8SenseKey[2] = 0x00;

                /* If CBW request for data phase, just return zero packet to end data phase */
                if (g_sCBW.dCBWDataTransferLength > 0) {
                    /* Data Phase, zero/short packet */
                    if ((g_sCBW.bmCBWFlags & 0x80) != 0) {
                        /* Data-In */
                        g_u8BulkState = BULK_IN;
                        _DRVUSB_TRIG_EP(2, 0);
                    }
                } else {
                    /* Status Phase */
                    g_u8BulkState = BULK_IN;
                    UsbBulkInAck();
                }

                return TRUE;
            }
        }
    } else if (g_u8BulkState == BULK_OUT) {
        switch (g_sCBW.u8OPCode) {
            case UFI_WRITE_10: {
                UsbWrite();
                return TRUE;
            }
        }
    }

    return TRUE;
}


//======================================================
// USB USB Event
//======================================================
void UsbUsb(uint32_t u32INTSTS)
{
    uint32_t u32STS = _DRVUSB_GET_EPSTS();

    if (u32INTSTS & INTSTS_SETUP) {
        /* Clear setup event */
        _DRVUSB_SET_EVENT_FLAG(INTSTS_SETUP);

        if (!UsbSetup()) {
            /* Setup error, stall the device */
            _DRVUSB_SET_CFGP(0, (CFGP_SSTALL | CFGP_CLRRDY));
            _DRVUSB_SET_CFGP(1, (CFGP_SSTALL | CFGP_CLRRDY));
        }
    } else if (u32INTSTS & INTSTS_EPTF0) {
        /* Ctrl In */
        _DRVUSB_SET_EVENT_FLAG(INTSTS_EPTF0);
        UsbInAck();
    } else if (u32INTSTS & INTSTS_EPTF1) {
        /* Ctrl Out */
        _DRVUSB_SET_EVENT_FLAG(INTSTS_EPTF1);
        UsbOutAck();
    } else if (u32INTSTS & INTSTS_EPTF2) {
        /* Bulk In */
        _DRVUSB_SET_EVENT_FLAG(INTSTS_EPTF2);
        g_u32OutToggle = u32STS;
        UsbBulkInAck();
    } else if (u32INTSTS & INTSTS_EPTF3) {
        /* Bulk Out */
        _DRVUSB_SET_EVENT_FLAG(INTSTS_EPTF3);

        if (g_u32OutToggle == (u32STS & STS3)) {
            _DRVUSB_TRIG_EP(3, MAX_PACKET_SIZE);
        } else {
            if (UsbBulkOutAck()) {
                g_u32OutToggle = u32STS & STS3;
            } else {
                /* stall */
                g_bCBWInvalid = TRUE;
                _DRVUSB_SET_CFGP(2, (CFGP_SSTALL | CFGP_CLRRDY));
                _DRVUSB_SET_CFGP(3, (CFGP_SSTALL | CFGP_CLRRDY));
            }
        }
    } else {
        /* Unknow event */
        _DRVUSB_SET_EVENT_FLAG(INTSTS_USB);
    }
}


void udcFlashInit(void)
{
    uint32_t u32StorageSize;
    uint32_t u32Addr = 0x8000;
    F_DATA_FLASH_LUN = MassLUN;
    MassLUN++;
    outp32(FMC_ISPCMD, 0x00);   /* ISP Read */
    /* To check APROM Size */
    outp32(FMC_ISPADR, 0x9000);
    outp32(FMC_ISPTRG, 0x01);
    __ISB();

    /* Check ISPFF flag to know whether erase OK or fail. */
    if (FMC->ISPCON & FMC_ISPCON_ISPFF_Msk) {
        FMC->ISPCON |= FMC_ISPCON_ISPFF_Msk;
        u32Addr = 0x8000;
    } else {
        u32Addr <<= 1;
    }

    if (FMC->DFBADR < u32Addr) {
        u32Addr = FMC->DFBADR;
    }

    //FMC->ISPCON.ISPEN = 0;
    /* Read Config0 value */
    outp32(FMC_ISPADR, 0x300000);
    outp32(FMC_ISPTRG, 0x01);
    __ISB();
    g_u8SecurityLockBit = inp32(FMC_ISPDAT) & BIT1;
    g_u32FlashSize = u32Addr;
    u32StorageSize = u32Addr + DATA_SECTOR_ADDRESS;
    g_dataFlashTotalSectors = u32StorageSize / UDC_SECTOR_SIZE;
    u8FormatData[19] = g_dataFlashTotalSectors & 0xFF;
    u8FormatData[20] = (uint8_t)(g_dataFlashTotalSectors >> 8);
}
