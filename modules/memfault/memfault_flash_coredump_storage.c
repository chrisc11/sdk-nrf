/*
 *
 * An implementation which will save Memfault coredumps (https://mflt.io/coredumps) to
 * internal flash on an NRF based MCU. Compared to saving coredumps in a RAM region,
 * this allows for more data to be captured and for it to persist across power losses.
 */

#include <pm_config.h>
#include <nrfx_nvmc.h>
#include <storage/flash_map.h>

#include "memfault/components.h"
#include "memfault/ports/buffered_coredump_storage.h"

void memfault_platform_coredump_storage_get_info(sMfltCoredumpStorageInfo *info)
{
	*info = (sMfltCoredumpStorageInfo) {
		.size = PM_MEMFAULT_STORAGE_SIZE,
	};
}

static bool prv_op_within_flash_bounds(uint32_t offset, size_t data_len)
{
	sMfltCoredumpStorageInfo info = { 0 };

	memfault_platform_coredump_storage_get_info(&info);
	return (offset + data_len) <= info.size;
}

/**
 * @note This is _only_ called when the system has crashed and a coredump is being saved.
 * memfault_coredump_read() is called when the data is being sent to the cloud for processing.
 */
bool memfault_platform_coredump_storage_read(uint32_t offset, void *data, size_t read_len)
{
	if (!prv_op_within_flash_bounds(offset, read_len)) {
		return false;
	}

	/* Note: internal flash is memory mapped so we can just memcpy it out */
	const uint32_t address = PM_MEMFAULT_STORAGE_ADDRESS + offset;

	memcpy(data, (void *)address, read_len);
	return true;
}

/**
 * @note: This is _only_ called when the system has crashed and a coredump is being saved.
 * We use the low level nrfx APIs because no RTOS primitives (i.e locks/semaphores) should
 * be used when the system is in this state.
 */
bool memfault_platform_coredump_storage_erase(uint32_t offset, size_t erase_size)
{
	if (!prv_op_within_flash_bounds(offset, erase_size)) {
		return false;
	}

	uint32_t page_size = nrfx_nvmc_flash_page_size_get();

	if ((offset % page_size) != 0) {
		return false;
	}

	for (size_t page = offset; page < erase_size; page += page_size) {
		const uint32_t address = PM_MEMFAULT_STORAGE_ADDRESS + page;

		nrfx_nvmc_page_erase(address);
	}

	return true;
}

/**
 * @note: This is _only_ called when the system has crashed and a coredump is being saved.
 * We use the low level nrfx APIs because no RTOS primitives (i.e locks/semaphores) should
 * be used when the system is in this state.
 */
bool memfault_platform_coredump_storage_buffered_write(sCoredumpWorkingBuffer *blk)
{
	if (!prv_op_within_flash_bounds(blk->write_offset, MEMFAULT_COREDUMP_STORAGE_WRITE_SIZE)) {
		return false;
	}

	const uint32_t start_addr = PM_MEMFAULT_STORAGE_ADDRESS;
	const uint32_t addr = start_addr + blk->write_offset;

	MEMFAULT_STATIC_ASSERT(MEMFAULT_COREDUMP_STORAGE_WRITE_SIZE % sizeof(uint32_t) == 0,
			       "Write buffer must be word aligned");
	nrfx_nvmc_words_write(addr, &blk->data[0],
			      MEMFAULT_COREDUMP_STORAGE_WRITE_SIZE / sizeof(uint32_t));
	return true;
}

/**
 * Note: while the system is running, flash writes for the nRF (soc_flash_nrf.c)
 * may be asynchronous so we use a static to track when a coredump clear request has
 * been issued
 */
static bool s_last_coredump_cleared;

bool memfault_coredump_read(uint32_t offset, void *data, size_t read_len)
{
	if (s_last_coredump_cleared) {
		/* we've already read the coredump, return nothing */
		memset(data, 0x0, read_len);
		return true;
	}

	return memfault_platform_coredump_storage_read(offset, data, read_len);
}


/**
 * @note: This is called after a coredump has been successfully sent to the cloud for processing
 * while the system is in normal operation mode.
 */
void memfault_platform_coredump_storage_clear(void)
{
	const struct flash_area *flash_area;
	int err = flash_area_open(PM_MEMFAULT_STORAGE_ID, &flash_area);

	if (err) {
		MEMFAULT_LOG_ERROR("Unable to open coredump storage: 0x%x", err);
		return;
	}

	uint32_t empty_word = 0x0;

	err = flash_area_write(flash_area, 0x0, &empty_word, sizeof(empty_word));
	if (err) {
		MEMFAULT_LOG_ERROR("Unable to clear storage: 0x%x", err);
		return;
	}

	s_last_coredump_cleared = true;
}
