#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrf_modem_gnss.h>
#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>

LOG_MODULE_REGISTER(gnss, LOG_LEVEL_INF);

static struct nrf_modem_gnss_pvt_data_frame last_pvt;
static K_SEM_DEFINE(pvt_data_sem, 0, 1);

static const char update_indicator[] = {'\\', '|', '/', '-'};

static void gnss_event_handler(int event)
{
	int retval;

	switch (event)
    {
        case NRF_MODEM_GNSS_EVT_PVT:
        {
            retval = nrf_modem_gnss_read(&last_pvt, sizeof(last_pvt), NRF_MODEM_GNSS_DATA_PVT);
            if (retval == 0)
            {
                k_sem_give(&pvt_data_sem);
            }
            break;
        }
        case NRF_MODEM_GNSS_EVT_FIX:
        case NRF_MODEM_GNSS_EVT_NMEA:
        case NRF_MODEM_GNSS_EVT_AGNSS_REQ:
        case NRF_MODEM_GNSS_EVT_BLOCKED:
        case NRF_MODEM_GNSS_EVT_UNBLOCKED:
        case NRF_MODEM_GNSS_EVT_PERIODIC_WAKEUP:
        case NRF_MODEM_GNSS_EVT_SLEEP_AFTER_TIMEOUT:
        case NRF_MODEM_GNSS_EVT_SLEEP_AFTER_FIX:
        case NRF_MODEM_GNSS_EVT_REF_ALT_EXPIRED:
        default:
        {
            break;
        }
	}
}

static void print_satellite_stats(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
	uint8_t tracked = 0;
	uint8_t in_fix = 0;
	uint8_t unhealthy = 0;

	for (int i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; ++i)
    {
		if (pvt_data->sv[i].sv > 0)
        {
			tracked++;
			if (pvt_data->sv[i].flags & NRF_MODEM_GNSS_SV_FLAG_USED_IN_FIX)
            {
				in_fix++;
			}
			if (pvt_data->sv[i].flags & NRF_MODEM_GNSS_SV_FLAG_UNHEALTHY)
            {
				unhealthy++;
			}
		}
	}

	printf("Tracking: %2d Using: %2d Unhealthy: %d\n", tracked, in_fix, unhealthy);
}

static void print_fix_data(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
	printf("Latitude:       %.06f\n", pvt_data->latitude);
	printf("Longitude:      %.06f\n", pvt_data->longitude);
	printf("Accuracy:       %.01f m\n", (double)pvt_data->accuracy);
	printf("Altitude:       %.01f m\n", (double)pvt_data->altitude);
	printf("Speed:          %.01f m/s\n", (double)pvt_data->speed);
	printf("Heading:        %.01f deg\n", (double)pvt_data->heading);
	printf("Date:           %04u-%02u-%02u\n", pvt_data->datetime.year, pvt_data->datetime.month, pvt_data->datetime.day);
	printf("Time (UTC):     %02u:%02u:%02u.%03u\n", pvt_data->datetime.hour, pvt_data->datetime.minute, pvt_data->datetime.seconds, pvt_data->datetime.ms);
	printf("PDOP:           %.01f\n", (double)pvt_data->pdop);
	printf("HDOP:           %.01f\n", (double)pvt_data->hdop);
	printf("VDOP:           %.01f\n", (double)pvt_data->vdop);
}

int main(void)
{
	int err;
	uint8_t cnt = 0;
	uint64_t fix_timestamp;

	LOG_INF("Starting Nomad");

	LOG_INF("Initializing modem library");
	err = nrf_modem_lib_init();
	if (err)
    {
		LOG_ERR("Modem library initialization failed, error: %d", err);
		return err;
	}
	LOG_INF("Modem library initialized");

	/* Activate GNSS only (no LTE). */
	if (lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_GNSS) != 0)
    {
		LOG_ERR("Failed to activate GNSS functional mode");
		return -1;
	}

    /* Set callback function for event interrupt. */
	if (nrf_modem_gnss_event_handler_set(gnss_event_handler) != 0)
    {
		LOG_ERR("Failed to set GNSS event handler");
		return -1;
	}

	/* Continuous tracking, no timeout. */
	if (nrf_modem_gnss_fix_retry_set(0) != 0)
    {
		LOG_ERR("Failed to set GNSS fix retry");
		return -1;
	}

	if (nrf_modem_gnss_fix_interval_set(1) != 0)
    {
		LOG_ERR("Failed to set GNSS fix interval");
		return -1;
	}

	if (nrf_modem_gnss_use_case_set(NRF_MODEM_GNSS_USE_CASE_MULTIPLE_HOT_START) != 0)
    {
		LOG_WRN("Failed to set GNSS use case");
	}

	if (nrf_modem_gnss_start() != 0)
    {
		LOG_ERR("Failed to start GNSS");
		return -1;
	}

	LOG_INF("GNSS started, searching for satellites...");
	fix_timestamp = k_uptime_get();

	for (;;)
    {
		if (k_sem_take(&pvt_data_sem, K_FOREVER) != 0)
        {
			continue;
		}

		printf("\033[1;1H");
		printf("\033[2J");
		print_satellite_stats(&last_pvt);

		if (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_DEADLINE_MISSED)
        {
			printf("GNSS operation blocked by LTE\n");
		}
		if (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_NOT_ENOUGH_WINDOW_TIME)
        {
			printf("Insufficient GNSS time windows\n");
		}

		printf("-----------------------------------\n");

		if (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID)
        {
			fix_timestamp = k_uptime_get();
			print_fix_data(&last_pvt);
		}
        else
        {
			printf("Seconds since last fix: %d\n", (uint32_t)((k_uptime_get() - fix_timestamp) / 1000));
			cnt++;
			printf("Searching [%c]\n", update_indicator[cnt % 4]);
		}
	}

	return 0;
}