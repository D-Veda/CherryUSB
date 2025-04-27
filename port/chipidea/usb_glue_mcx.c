/*
 * Copyright (c) 2024, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "usbd_core.h"
#include "fsl_common.h"

/*! @brief USB controller ID */
typedef enum _usb_controller_index {
    kUSB_ControllerKhci0 = 0U, /*!< KHCI 0U */
    kUSB_ControllerKhci1 = 1U, /*!< KHCI 1U, Currently, there are no platforms which have two KHCI IPs, this is reserved
                                  to be used in the future. */
    kUSB_ControllerEhci0 = 2U, /*!< EHCI 0U */
    kUSB_ControllerEhci1 = 3U, /*!< EHCI 1U */
} usb_controller_index_t;

#define USB_DEVICE_CONFIG_EHCI 1

/* USB PHY condfiguration */
#define BOARD_USB_PHY_D_CAL     (0x04U)
#define BOARD_USB_PHY_TXCAL45DP (0x07U)
#define BOARD_USB_PHY_TXCAL45DM (0x07U)
#define BOARD_XTAL0_CLK_HZ      24000000U /*!< Board xtal0 frequency in Hz */

#if ((defined FSL_FEATURE_SOC_USBPHY_COUNT) && (FSL_FEATURE_SOC_USBPHY_COUNT > 0U))
typedef struct _usb_phy_config_struct
{
    uint8_t D_CAL;     /* Decode to trim the nominal 17.78mA current source */
    uint8_t TXCAL45DP; /* Decode to trim the nominal 45-Ohm series termination resistance to the USB_DP output pin */
    uint8_t TXCAL45DM; /* Decode to trim the nominal 45-Ohm series termination resistance to the USB_DM output pin */
} usb_phy_config_struct_t;

void *USB_EhciPhyGetBase(uint8_t controllerId)
{
    void *usbPhyBase = NULL;
#if ((defined FSL_FEATURE_SOC_USBPHY_COUNT) && (FSL_FEATURE_SOC_USBPHY_COUNT > 0U))
#if defined(USBPHY_STACK_BASE_ADDRS)
    uint32_t usbphy_base[] = USBPHY_STACK_BASE_ADDRS;
#else
    uint32_t usbphy_base[] = USBPHY_BASE_ADDRS;
#endif
    uint32_t *temp;
    if (controllerId < (uint8_t)kUSB_ControllerEhci0)
    {
        return NULL;
    }

    if ((controllerId == (uint8_t)kUSB_ControllerEhci0) || (controllerId == (uint8_t)kUSB_ControllerEhci1))
    {
        controllerId = controllerId - (uint8_t)kUSB_ControllerEhci0;
    }
    else
    {
        return NULL;
    }

    if (controllerId < (sizeof(usbphy_base) / sizeof(usbphy_base[0])))
    {
        temp       = (uint32_t *)usbphy_base[controllerId];
        usbPhyBase = (void *)temp;
    }
    else
    {
        return NULL;
    }
#endif
    return usbPhyBase;
}

void USB_EhciPhyInit(uint8_t controllerId, uint32_t freq, usb_phy_config_struct_t *phyConfig)
{
#if ((defined FSL_FEATURE_SOC_USBPHY_COUNT) && (FSL_FEATURE_SOC_USBPHY_COUNT > 0U))
    USBPHY_Type *usbPhyBase;

    usbPhyBase = (USBPHY_Type *)USB_EhciPhyGetBase(controllerId);
    if (NULL == usbPhyBase)
    {
        return;
    }

#if ((defined FSL_FEATURE_SOC_ANATOP_COUNT) && (FSL_FEATURE_SOC_ANATOP_COUNT > 0U))
    ANATOP->HW_ANADIG_REG_3P0.RW =
        (ANATOP->HW_ANADIG_REG_3P0.RW &
         (~(ANATOP_HW_ANADIG_REG_3P0_OUTPUT_TRG(0x1F) | ANATOP_HW_ANADIG_REG_3P0_ENABLE_ILIMIT_MASK))) |
        ANATOP_HW_ANADIG_REG_3P0_OUTPUT_TRG(0x17) | ANATOP_HW_ANADIG_REG_3P0_ENABLE_LINREG_MASK;
    ANATOP->HW_ANADIG_USB2_CHRG_DETECT.SET =
        ANATOP_HW_ANADIG_USB2_CHRG_DETECT_CHK_CHRG_B_MASK | ANATOP_HW_ANADIG_USB2_CHRG_DETECT_EN_B_MASK;
#endif

#if (defined USB_ANALOG)
    USB_ANALOG->INSTANCE[controllerId - (uint8_t)kUSB_ControllerEhci0].CHRG_DETECT_SET =
        USB_ANALOG_CHRG_DETECT_CHK_CHRG_B(1) | USB_ANALOG_CHRG_DETECT_EN_B(1);
#endif

#if ((!(defined FSL_FEATURE_SOC_CCM_ANALOG_COUNT)) && (!(defined FSL_FEATURE_SOC_ANATOP_COUNT)))

    usbPhyBase->TRIM_OVERRIDE_EN = 0x001fU; /* override IFR value */
#endif
    usbPhyBase->CTRL |= USBPHY_CTRL_SET_ENUTMILEVEL2_MASK; /* support LS device. */
    usbPhyBase->CTRL |= USBPHY_CTRL_SET_ENUTMILEVEL3_MASK; /* support external FS Hub with LS device connected. */
    /* PWD register provides overall control of the PHY power state */
    usbPhyBase->PWD = 0U;
    if (NULL != phyConfig)
    {
        /* Decode to trim the nominal 17.78mA current source for the High Speed TX drivers on USB_DP and USB_DM. */
        usbPhyBase->TX =
            ((usbPhyBase->TX & (~(USBPHY_TX_D_CAL_MASK | USBPHY_TX_TXCAL45DM_MASK | USBPHY_TX_TXCAL45DP_MASK))) |
             (USBPHY_TX_D_CAL(phyConfig->D_CAL) | USBPHY_TX_TXCAL45DP(phyConfig->TXCAL45DP) |
              USBPHY_TX_TXCAL45DM(phyConfig->TXCAL45DM)));
    }
#endif

    return;
}

#endif

#if (defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U))
void USB1_HS_IRQHandler(void)
{
    extern void USBD_IRQHandler(uint8_t busid);
    USBD_IRQHandler(0);
}
#endif

void USB_ClockInit(void)
{
#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)
    usb_phy_config_struct_t phyConfig = {
        BOARD_USB_PHY_D_CAL,
        BOARD_USB_PHY_TXCAL45DP,
        BOARD_USB_PHY_TXCAL45DM,
    };
#endif
#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)
    SPC0->ACTIVE_VDELAY = 0x0500;
    /* Change the power DCDC to 1.8v (By deafult, DCDC is 1.8V), CORELDO to 1.1v (By deafult, CORELDO is 1.0V) */
    SPC0->ACTIVE_CFG &= ~SPC_ACTIVE_CFG_CORELDO_VDD_DS_MASK;
    SPC0->ACTIVE_CFG |= SPC_ACTIVE_CFG_DCDC_VDD_LVL(0x3) | SPC_ACTIVE_CFG_CORELDO_VDD_LVL(0x3) |
                        SPC_ACTIVE_CFG_SYSLDO_VDD_DS_MASK | SPC_ACTIVE_CFG_DCDC_VDD_DS(0x2u);
    /* Wait until it is done */
    while (SPC0->SC & SPC_SC_BUSY_MASK)
        ;
    if (0u == (SCG0->LDOCSR & SCG_LDOCSR_LDOEN_MASK)) {
        SCG0->TRIM_LOCK = 0x5a5a0001U;
        SCG0->LDOCSR |= SCG_LDOCSR_LDOEN_MASK;
        /* wait LDO ready */
        while (0U == (SCG0->LDOCSR & SCG_LDOCSR_VOUT_OK_MASK))
            ;
    }
    SYSCON->AHBCLKCTRLSET[2] |= SYSCON_AHBCLKCTRL2_USB_HS_MASK | SYSCON_AHBCLKCTRL2_USB_HS_PHY_MASK;
    SCG0->SOSCCFG &= ~(SCG_SOSCCFG_RANGE_MASK | SCG_SOSCCFG_EREFS_MASK);
    /* xtal = 20 ~ 30MHz */
    SCG0->SOSCCFG = (1U << SCG_SOSCCFG_RANGE_SHIFT) | (1U << SCG_SOSCCFG_EREFS_SHIFT);
    SCG0->SOSCCSR |= SCG_SOSCCSR_SOSCEN_MASK;
    while (1) {
        if (SCG0->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK) {
            break;
        }
    }
    SYSCON->CLOCK_CTRL |= SYSCON_CLOCK_CTRL_CLKIN_ENA_MASK | SYSCON_CLOCK_CTRL_CLKIN_ENA_FM_USBH_LPT_MASK;
    CLOCK_EnableClock(kCLOCK_UsbHs);
    CLOCK_EnableClock(kCLOCK_UsbHsPhy);
    CLOCK_EnableUsbhsPhyPllClock(kCLOCK_Usbphy480M, 24000000U);
    CLOCK_EnableUsbhsClock();
    USB_EhciPhyInit(kUSB_ControllerEhci0, BOARD_XTAL0_CLK_HZ, &phyConfig);
#endif
#if defined(USB_DEVICE_CONFIG_KHCI) && (USB_DEVICE_CONFIG_KHCI > 0U)
    CLOCK_AttachClk(kCLK_48M_to_USB0);
    CLOCK_EnableClock(kCLOCK_Usb0Ram);
    CLOCK_EnableClock(kCLOCK_Usb0Fs);
    CLOCK_EnableUsbfsClock();
#endif
}

void usb_dc_low_level_init(uint8_t busid)
{
    USB_ClockInit();
    /* Install isr, set priority, and enable IRQ. */
    NVIC_SetPriority((IRQn_Type)USB1_HS_IRQn, 3);
    EnableIRQ((IRQn_Type)USB1_HS_IRQn);
}

void usb_dc_low_level_deinit(uint8_t busid)
{
    DisableIRQ((IRQn_Type)USB1_HS_IRQn);
}