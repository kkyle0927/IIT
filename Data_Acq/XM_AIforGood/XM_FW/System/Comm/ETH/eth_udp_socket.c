/**
  ******************************************************************************
  * @file    eth_udp_socket.c
  * @author  HyundoKim
  * @brief   AGR_DOP over UDP — LwIP UDP 소켓 인프라 구현 (Rev2.0 Ethernet)
  *
  *          3개의 UDP PCB(Protocol Control Block)를 생성하고 바인드:
  *          - PDO  (Port 5001): Real-time cyclic data
  *          - SDO  (Port 5002): Request-response configuration
  *          - EMCY (Port 5003): Emergency notifications
  *
  * @version 1.0
  * @date    2026-03-02
  *
  * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "eth_udp_socket.h"

#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/ip4_addr.h"

#include <string.h>

/* Private defines -----------------------------------------------------------*/

/** @brief UDP 포트 테이블 (채널 순서와 동일) */
static const uint16_t s_port_table[ETH_UDP_CH_COUNT] = {
    ETH_UDP_PORT_PDO,   /* CH_PDO  = 0 → 5001 */
    ETH_UDP_PORT_SDO,   /* CH_SDO  = 1 → 5002 */
    ETH_UDP_PORT_EMCY,  /* CH_EMCY = 2 → 5003 */
};

/* Private types -------------------------------------------------------------*/

/** @brief 채널별 소켓 상태 */
typedef struct {
    struct udp_pcb  *pcb;       /**< LwIP UDP PCB */
    EthUdpRxCallback_t rx_cb;   /**< 수신 콜백 */
} EthUdpChannelCtx_t;

/* Private variables ---------------------------------------------------------*/

static EthUdpChannelCtx_t s_channels[ETH_UDP_CH_COUNT];
static bool s_initialized = false;
static ip_addr_t s_am_ip;  /**< 기본 AM(Jetson) IP 주소 */

/* Private function prototypes -----------------------------------------------*/
static void _UdpRecvCallback(void *arg, struct udp_pcb *pcb,
                              struct pbuf *p, const ip_addr_t *addr, u16_t port);

/* Public functions ----------------------------------------------------------*/

bool EthUdpSocket_Init(void)
{
    if (s_initialized)
    {
        return true;  /* 이미 초기화됨 */
    }

    memset(s_channels, 0, sizeof(s_channels));

    /* AM(Jetson) 기본 IP 주소 설정 */
    IP4_ADDR(&s_am_ip, ETH_UDP_AM_IP_0, ETH_UDP_AM_IP_1,
             ETH_UDP_AM_IP_2, ETH_UDP_AM_IP_3);

    /* 3개 채널 UDP PCB 생성 및 바인드 */
    for (int ch = 0; ch < ETH_UDP_CH_COUNT; ch++)
    {
        s_channels[ch].pcb = udp_new();
        if (s_channels[ch].pcb == NULL)
        {
            return false;
        }

        err_t err = udp_bind(s_channels[ch].pcb, IP_ADDR_ANY, s_port_table[ch]);
        if (err != ERR_OK)
        {
            udp_remove(s_channels[ch].pcb);
            s_channels[ch].pcb = NULL;
            return false;
        }

        /* 수신 콜백 등록 (arg = 채널 인덱스) */
        udp_recv(s_channels[ch].pcb, _UdpRecvCallback, (void *)(intptr_t)ch);
    }

    s_initialized = true;
    return true;
}

void EthUdpSocket_RegisterRxCallback(EthUdpChannel_t channel,
                                      EthUdpRxCallback_t callback)
{
    if (channel < ETH_UDP_CH_COUNT)
    {
        s_channels[channel].rx_cb = callback;
    }
}

int EthUdpSocket_Send(EthUdpChannel_t channel,
                       const uint8_t *data, uint16_t len)
{
    return EthUdpSocket_SendTo(channel, data, len,
                                s_am_ip.addr, s_port_table[channel]);
}

int EthUdpSocket_SendTo(EthUdpChannel_t channel,
                         const uint8_t *data, uint16_t len,
                         uint32_t dest_ip, uint16_t dest_port)
{
    if (!s_initialized || channel >= ETH_UDP_CH_COUNT)
    {
        return -1;
    }

    struct udp_pcb *pcb = s_channels[channel].pcb;
    if (pcb == NULL)
    {
        return -2;
    }

    /* pbuf 할당 (PBUF_TRANSPORT: UDP 헤더 공간 예약) */
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    if (p == NULL)
    {
        return -3;
    }

    /* 데이터 복사 */
    memcpy(p->payload, data, len);

    /* 목적지 IP 구성 */
    ip_addr_t dest;
    dest.addr = dest_ip;

    /* UDP 전송 */
    err_t err = udp_sendto(pcb, p, &dest, dest_port);
    pbuf_free(p);

    return (err == ERR_OK) ? 0 : -4;
}

bool EthUdpSocket_IsReady(void)
{
    return s_initialized;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  LwIP UDP 수신 콜백 (모든 채널 공유)
  * @note   LwIP tcpip_thread 컨텍스트에서 호출됨.
  *         pbuf는 이 함수 종료 전에 free 해야 함.
  */
static void _UdpRecvCallback(void *arg, struct udp_pcb *pcb,
                              struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    (void)pcb;

    if (p == NULL)
    {
        return;
    }

    EthUdpChannel_t ch = (EthUdpChannel_t)(intptr_t)arg;

    if (ch < ETH_UDP_CH_COUNT && s_channels[ch].rx_cb != NULL)
    {
        /* 체인된 pbuf를 단일 연속 버퍼로 복사 */
        if (p->tot_len <= 1472U && p->tot_len > 0)
        {
            uint8_t rx_buf[1472];
            uint16_t copied = pbuf_copy_partial(p, rx_buf, p->tot_len, 0);
            if (copied > 0)
            {
                s_channels[ch].rx_cb(ch, rx_buf, copied, addr->addr, port);
            }
        }
    }

    pbuf_free(p);
}
