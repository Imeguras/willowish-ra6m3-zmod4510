#include "communication_thread0.h"
#include "server_certificate.h"
#include  "nx_api.h"
#include  <nx_crypto.h>
#include  <nx_secure_tls_api.h>
#include  <nx_secure_x509.h>

/* Packet pool instance (If this is a Trustzone part, the memory must be placed in Non-secure memory). */
NX_PACKET_POOL g_packet_pool0;

#if defined(ETHER_BUFFER_PLACE_IN_SECTION)
uint8_t g_packet_pool0_pool_memory[G_PACKET_POOL0_PACKET_NUM * (G_PACKET_POOL0_PACKET_SIZE + sizeof(NX_PACKET))] BSP_ALIGN_VARIABLE(4) ETHER_BUFFER_PLACE_IN_SECTION;
#elif defined(WIFI_BUFFER_PLACE_IN_SECTION)
uint8_t g_packet_pool0_pool_memory[G_PACKET_POOL0_PACKET_NUM * (G_PACKET_POOL0_PACKET_SIZE + sizeof(NX_PACKET))] BSP_ALIGN_VARIABLE(4) WIFI_BUFFER_PLACE_IN_SECTION;
#endif

/* Quick Setup for g_packet_pool0.
 * - nx_system_initialize() must be called before calling this function.
 */
void g_packet_pool0_quick_setup()
{
    /* Create the packet pool. */
    UINT status = nx_packet_pool_create(&g_packet_pool0,
                "g_packet_pool0 Packet Pool",
                G_PACKET_POOL0_PACKET_SIZE,
                &g_packet_pool0_pool_memory[0],
                G_PACKET_POOL0_PACKET_NUM * (G_PACKET_POOL0_PACKET_SIZE + sizeof(NX_PACKET)));
    assert(NX_SUCCESS == status);
}

/* IP instance */
NX_IP g_ip0;

/* Stack memory for g_ip0. */
uint8_t g_ip0_stack_memory[G_IP0_TASK_STACK_SIZE] BSP_PLACE_IN_SECTION(".stack.g_ip0") BSP_ALIGN_VARIABLE(BSP_STACK_ALIGNMENT);

/* ARP cache memory for g_ip0. */
uint8_t g_ip0_arp_cache_memory[G_IP0_ARP_CACHE_SIZE] BSP_ALIGN_VARIABLE(4);

/* Quick setup for g_ip0.
 * - g_packet_pool0 must be setup before calling this function
 *     (See Developer Assistance -> g_ip0 -> g_packet_pool0 -> Quick Setup).
 * - nx_system_initialize() must be called before calling this function.
 * - If using a NetX Duo Wireless Driver variant the quick setup for wireless must be setup (so it can connect to a network) before calling this function
 *     (See Developer Assistance -> g_ip0 -> NetX Duo Wireless Driver -> Quick Setup)
 * - If using NetX Duo with the Embedded Wireless Framework (EWF) and Cellular products then please see the FSP user manual for what to setup prior to calling this function.
 */
void g_ip0_quick_setup()
{
    UINT status;

    /* Create the ip instance. */
    status = nx_ip_create(&g_ip0,
                "g_ip0 IP Instance",
                G_IP0_ADDRESS,
                G_IP0_SUBNET_MASK,
                &g_packet_pool0,
                G_IP0_NETWORK_DRIVER,
                &g_ip0_stack_memory[0],
                G_IP0_TASK_STACK_SIZE,
                G_IP0_TASK_PRIORITY);
    assert(NX_SUCCESS == status);

    /* If using Embedded Wireless Framework then uncomment this out to set the adapter_ptr. See the FSP user manual for more details. */
    // g_ip0.nx_ip_reserved_ptr = adapter_ptr;

    /* Set the gateway address if it is configured. Make sure this is set if using the Embedded Wireless Framework! */
    if(IP_ADDRESS(0, 0, 0, 0) != G_IP0_GATEWAY_ADDRESS)
    {
        status = nx_ip_gateway_address_set(&g_ip0, G_IP0_GATEWAY_ADDRESS);
        assert(NX_SUCCESS == status);
    }

    status = nx_arp_enable(&g_ip0, &g_ip0_arp_cache_memory[0], G_IP0_ARP_CACHE_SIZE);
    assert(NX_SUCCESS == status);



    status = nx_tcp_enable(&g_ip0);
    assert(NX_SUCCESS == status);

    status = nx_udp_enable(&g_ip0);
    assert(NX_SUCCESS == status);

    status = nx_icmp_enable(&g_ip0);
    assert(NX_SUCCESS == status);

    // status = nx_ip_fragment_enable(&g_ip0);
    // assert(NX_SUCCESS == status);

    // status = nx_igmp_enable(&g_ip0);
    // assert(NX_SUCCESS == status);

    /* Wait for the link to be enabled. */
    ULONG current_state;
    status = nx_ip_status_check(&g_ip0, NX_IP_LINK_ENABLED, &current_state, 1000U);
    assert(NX_SUCCESS == status);
    assert(NX_IP_LINK_ENABLED == current_state);
}
NX_SECURE_X509_CERT trusted_certificate;
#define NX_WEB_HTTP_SESSION_MAX 2
/* Define TLS data for HTTPS. */
CHAR crypto_metadata[8928 * NX_WEB_HTTP_SESSION_MAX];
UCHAR tls_packet_buffer[16500];
NX_SECURE_X509_CERT remote_certificate, remote_issuer;
UCHAR remote_cert_buffer[2000];
UCHAR remote_issuer_buffer[2000];
extern const NX_SECURE_TLS_CRYPTO nx_crypto_tls_ciphers;

/* Web HTTP Client instance. */
NX_WEB_HTTP_CLIENT  g_web_http_client0;

/* Quick setup for g_web_http_client0.
 * - g_ip0 must be setup before calling this function
 *     (See Developer Assistance -> g_web_http_client0 -> g_ip0 -> Quick Setup).
 * - g_packet_pool0 must be setup before calling this function
 *     (See Developer Assistance -> g_web_http_client0 -> g_packet_pool0 -> Quick Setup).
 * - nx_system_initialize() must be called before calling this function.
 */
void g_web_http_client0_quick_setup()
{
    /* Create the Web HTTP Client instance. */
    UINT status = nx_web_http_client_create(&g_web_http_client0,
                        "g_web_http_client0",
                        &g_ip0,
                        &g_packet_pool0,
                        G_WEB_HTTP_CLIENT0_WINDOW_SIZE);
    assert(NX_SUCCESS == status);
}
#define MIGRATION_SIZE 100
rm_zmod4xxx_oaq_2nd_data_t brap_data[MIGRATION_SIZE];

UINT tls_setup_callback(NX_WEB_HTTP_CLIENT *client_ptr,NX_SECURE_TLS_SESSION *tls_session){
    UINT status;

    /* Initialize and create TLS session. */
    nx_secure_tls_session_create(tls_session, &nx_crypto_tls_ciphers,crypto_metadata, sizeof(crypto_metadata));

    /* Allocate space for packet reassembly. */
    nx_secure_tls_session_packet_buffer_set(tls_session, tls_packet_buffer,sizeof(tls_packet_buffer));


    /* Add a CA Certificate to our trusted store for verifying incoming server
        certificates. */
    nx_secure_x509_certificate_initialize(&trusted_certificate,(UCHAR *) g_server_certificate, strlen(g_server_certificate),
            NX_NULL, 0, NULL, 0,NX_SECURE_X509_KEY_TYPE_NONE);
    nx_secure_tls_trusted_certificate_add(tls_session, &trusted_certificate);

    /* Need to allocate space for the certificate coming in from the remote host. */
    nx_secure_tls_remote_certificate_allocate(tls_session, &remote_certificate,
        remote_cert_buffer, sizeof(remote_cert_buffer));
    nx_secure_tls_remote_certificate_allocate(tls_session,
        &remote_issuer, remote_issuer_buffer,
        sizeof(remote_issuer_buffer));

    return(NX_SUCCESS);
 }

#define HOST_HEADER "Host"
#define HOST_HEADER_SIZE 5
/* Communication Thread entry function */
void communication_thread0_entry(void){
    int __index=0;
    nx_system_initialize();
    g_packet_pool0_quick_setup();

    g_ip0_quick_setup();
    g_web_http_client0_quick_setup();
    /* Setup the platform; initialize the SCE and the TRNG */
    uint32_t err = nx_crypto_initialize();
    assert(NX_CRYPTO_SUCCESS == err);

    nx_secure_tls_initialize();
    //_nx_web_http_client_get_secure_start(NX_WEB_HTTP_CLIENT *client_ptr, NXD_ADDRESS *server_ip, UINT server_port, CHAR *resource,CHAR *host, CHAR *username, CHAR *password,UINT (*tls_setup)(NX_WEB_HTTP_CLIENT *client_ptr, NX_SECURE_TLS_SESSION *),ULONG wait_option);
    //nx_web_http_client_secure_connect(&g_web_http_client0, &end_point, 443,tls_setup_callback, TX_WAIT_FOREVER);

    volatile UINT _t = NX_SUCCESS;


    _t =  nx_web_http_client_get_secure_start(&g_web_http_client0, &end_point, 443,HOST_END_POINT,(CHAR *)"/api/v1/User","", "", tls_setup_callback, TX_WAIT_FOREVER);
    //apparently microsoft thought putting a HOST header on this made sense(which although it eases the process with azure or any other cloud/vps/ any serious provider, they underestimate the lack of money there is);

   //_f = nx_web_http_client_request_header_add(&g_web_http_client0, (CHAR *)HOST_HEADER, HOST_HEADER_SIZE,(CHAR *) HOST_END_POINT, HOST_END_POINT_SIZE(), TX_WAIT_FOREVER);
   // status =  nx_web_http_client_response_body_get(&my_client, &my_packet, 20);


    while (1){
        while(__index<=MIGRATION_SIZE || data_aqi_queue.tx_queue_enqueued>0 ){
            tx_queue_receive(&data_aqi_queue,(rm_zmod4xxx_oaq_2nd_data_t *)&brap_data[__index], TX_WAIT_FOREVER);
        }

        //nx_http_client_put_start(&my_client, &server_ip_address, "/client_test.htm", "name", "password", 103, 500);

        __index=0;
		



        tx_thread_sleep (1);
    }
}
