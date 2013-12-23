/* Sample flow for validating 2level page tables - Extracted from test code base */


#define  START_ADDR_DDR               (0x80000000)
#define  START_ADDR_DEV               (0x44000000)
#define  START_ADDR_OCMC              (0x40300000)

#define  SIZE_BYTES_DDR               (128*1024*1024)
#define  SIZE_BYTES_DEV               (960*1024*1024)
#define  SIZE_BYTES_OCMC              (1*1024*1024)



/** \brief Defines a statically allocated Level 1 translation table. Need to be
 initialised to zero.
 */
#pragma DATA_ALIGN(pageTable1, MMU_FIRSTLEVEL_ALIGN_SIZE);
static volatile uint32_t pageTable1[MMU_FIRSTLEVEL_NUM_32BIT_ENTRY] = {0};

/** \brief Defines a statically allocated Level 2 page table. Need to be
 initialised to zero.
 */
#pragma DATA_ALIGN(pageTable2, MMU_SECONDLEVEL_ALIGN_SIZE);
static volatile uint32_t pageTable2[MMU_SECONDLEVEL_NUM_32BIT_ENTRY] = {0};

void MMUConfigAndEnablePages(void)
{
	int32_t status;
	uint32_t *lastAddr;

		mmuMemRegionConfig_t regionDdr =
					{
						START_ADDR_DDR,
						(128*1024/4), /* Number of pages */
						4096, /* Page size */
						MMU_MEM_ATTR_NORMAL_NON_SHAREABLE,
						MMU_CACHE_POLICY_WB_WA, /* Inner */
						MMU_CACHE_POLICY_WB_WA, /* Outer */
						MMU_ACCESS_CTRL_PRV_RW_USR_RW,
						FALSE /* Non Secure memory */
					};

		/** \brief Define OCMC RAM region. */
		/* TODO: Get OCMC RAM base address and size from chipdb. */
		mmuMemRegionConfig_t regionOcmc =
					{
						START_ADDR_OCMC,
						(1*1024/4), /* Number of pages */
						4096, /* Page size */
						MMU_MEM_ATTR_NORMAL_NON_SHAREABLE,
						MMU_CACHE_POLICY_WT_NOWA, /* Inner */
						MMU_CACHE_POLICY_WB_WA, /* Outer */
						MMU_ACCESS_CTRL_PRV_RW_USR_RW,
						FALSE /* Non Secure memory */
					};

		/** \brief Define Device Memory Region. The region between OCMC and DDR is
		 *         configured as device memory, with R/W access in user/privileged
		 *         modes. Also, the region is marked 'Execute Never'.
		 */
		mmuMemRegionConfig_t regionDev =
					{
						START_ADDR_DEV,
						(960*1024/4), /* Number of pages */
						4096, /* Page size */
						MMU_MEM_ATTR_DEVICE_SHAREABLE,
						MMU_CACHE_POLICY_WB_WA, /* Inner - Invalid here */
						MMU_CACHE_POLICY_WB_WA, /* Outer - Invalid here */
						MMU_ACCESS_CTRL_PRV_RW_USR_RW,
						FALSE /* Non Secure memory */
					};
		   /* Initialize the MMU */
			MMUInitEx();

		    /* Map the defined regions */
			status = MMUMemRegionMapEx(&regionDdr, (uint32_t*)pageTable1, (uint32_t*)pageTable2, TRUE, &lastAddr);
			if(status != S_PASS)
			{
				exit(-1);
			}
		    status = MMUMemRegionMapEx(&regionOcmc, (uint32_t*)pageTable1, (uint32_t*)pageTable2, TRUE, &lastAddr);
			if(status != S_PASS)
			{
				exit(-1);
			}
		    status = MMUMemRegionMapEx(&regionDev, (uint32_t*)pageTable1, (uint32_t*)pageTable2, TRUE, &lastAddr);
			if(status != S_PASS)
			{
				exit(-1);
			}

		    /* Now Safe to enable MMU */
		    MMUEnable((unsigned int*)pageTable1);
}
