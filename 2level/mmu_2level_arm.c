/**
 *  \file   mmu_2level_arm.c
 *
 *  \brief  APIs for configuring MMU for small page usage in ARM. 
 *          API's are provided for initializing MMU and configuring 
 *          memory regions.
 *
 *  \copyright Copyright (C) 2014 Texas Instruments Incorporated - 
 *             http://www.ti.com/
 */

/**
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "types.h"
#include "mmu.h"
#include "cp15.h"
#include "error.h"
#include "misc.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** Alignment for Page Table - shall be aligned to 16KB boundary. */
#define MMU_PAGETABLE_ALIGN_SIZE                          (16U*1024U)

#define MMU_PAGETABLE_ENTRY_FAULT                         (0x00U)
#define MMU_PAGEBOUND_SHIFT                               (20U)

/** \brief Page table base address mask for first level descriptors. */
#define MMU_PAGETABLE_ADDR_MASK                           (0xFFF00000U)
/** \brief Page table base address shift for first level descriptors. */
#define MMU_PAGETABLE_ADDR_SHIFT                          (20U)
/** \brief Section base address. */
#define MMU_PGADDR_SECTION                                (0xFFFU)
/** \brief SSuper-section base address. Ignoring extended base address. */
#define MMU_PGADDR_SUPER_SECTION                          (0xFF0U)

/** \brief Page type mask for section & super-section. */
#define MMU_PGTYPE_SECTION_MASK                           (0x00040002U)
/** \brief Page type - Section which is 1MB. */
#define MMU_PGTYPE_SECTION                                (0x00000002U)
/** \brief Page type - Super-section which is 16MB. */
#define MMU_PGTYPE_SUPER_SECTION                          (0x00040002U)
/** \brief Page type - Small page 4kB in L1 */
#define MMU_PGTYPE_SMALL_PAGE_L1                          (0x00000001U)
/** \brief Page type - Small page 4kB in L2 */
#define MMU_PGTYPE_SMALL_PAGE_L2                          (0x00000002U)

/** \brief Memory Type is normal - Non Shareable */
#define MMU_MEMTYPE_NORMAL_NON_SHAREABLE                  (0x00004000U)
/** \brief Memory Type is normal - Shareable */
#define MMU_MEMTYPE_NORMAL_SHAREABLE                      (0x00014000U)
/** \brief Inner cache policy bit shift */
#define MMU_SMALLPAGE_INNER_CACHE_POLICY_SHIFT                      (2U)
/** \brief Outer cache policy bit shift */
#define MMU_SMALLPAGE_OUTER_CACHE_POLICY_SHIFT                      (6U)

/** \brief Non secure bit shift */
#define MMU_NON_SECURE_MASK_SMALLPAGE                         (0x0008U)

/** \brief Offset from start of linear L2 table in 1:1 mapping */
#define GENERATE_L2_SMALLPAGE_OFFSET(regionStart, mbindex) \
									( (((regionStart) >> 20) << 8) + \
										((mbindex) << 8) )
/** \brief Entry in linear L2 table in 1:1 mapping */
#define GENERATE_L2_SMALLPAGE_BASE_ENTRY(regionStart, mbindex, kbindex) \
													( ((regionStart)+ \
													((mbindex) << 20) + \
													((kbindex) << 12) ) & \
														0xFFFFF000 )

/* \brief Number of entries in Level1 translation table */
#define MMU_FIRSTLEVEL_NUM_32BIT_ENTRY (4096)

/* \brief Number of entries in each Level 2 table */
#define MMU_SECONDLEVEL_32BIT_ENTRIES_PER_TABLE (256)

/* \brief Total number of entries in Level 2 for 1 process */
#define MMU_SECONDLEVEL_NUM_32BIT_ENTRY \
		((MMU_SECONDLEVEL_32BIT_ENTRIES_PER_TABLE)* \
				(MMU_FIRSTLEVEL_NUM_32BIT_ENTRY))

/* \brief Alignment for 4k entry L1 table */
#define MMU_FIRSTLEVEL_ALIGN_SIZE (4096*4)

/* \brief Alignment for each 256 entry L2 table */
#define MMU_SECONDLEVEL_ALIGN_SIZE (256*4)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                            Local Variables                                 */
/* ========================================================================== */
/** \brief Defines a static  information table for access control of Pages
 *         permissions defined in ARM. The order of entries in the table
 *                   should be as per the enum "mmuAccessCtrl_t". These 
 *                   values represent values in (level 2) page table entry.
 */
static const uint32_t gMmuAccessCtrlForSmallPage[] =
                {
                        0x00000000U, /* MMU_ACCESS_CTRL_PRV_NA_USR_NA */
                        0x0000010U, /* MMU_ACCESS_CTRL_PRV_RW_USR_NA */
                        0x0000020U, /* MMU_ACCESS_CTRL_PRV_RW_USR_RO */
                        0x0000030U, /* MMU_ACCESS_CTRL_PRV_RW_USR_RW */
                        0x0000210U, /* MMU_ACCESS_CTRL_PRV_RO_USR_NA */
                        0x0000220U, /* MMU_ACCESS_CTRL_PRV_RO_USR_RO */
                        0x00000230U  /* MMU_ACCESS_CTRL_PRV_NO_EXEC */
                };


/** \brief Defines a static  information table for memory attributes of Pages
 *         defined in ARM. The order of entries in the table should be as per
 *         the enum "mmuMemAttr_t". These values represent the configuration
 *                   in the (level 2) page table entry.
 */
static const uint32_t gMmuMemAttrForSmallPage[] =
                {
                        0x00000000U, /* MMU_MEM_ATTR_SO_SHAREABLE */
                        0x00000040U, /* MMU_MEM_ATTR_DEVICE_SHAREABLE */
                        0x00000080U, /* MMU_MEM_ATTR_DEVICE_NON_SHAREABLE */
                        0x000500U, /* MMU_MEM_ATTR_NORMAL_SHAREABLE_NON_CACHEABLE */
                        0x0000100U  /* MMU_MEM_ATTR_NORMAL_NON_SHAREABLE_NON_CACHEABLE */
                };

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
void MMUInitEx()
{
   /* Invalidate the TLB entries */
    CP15TlbInvalidate();

    /* Set domain access rights */
    CP15DomainAccessClientSet();

    /* Disable TEX remapping, Access Flag usage and alignment check */
    CP15ControlFeatureDisable( CP15_CONTROL_TEXREMAP
                               | CP15_CONTROL_ACCESSFLAG
                               | CP15_CONTROL_ALIGN_CHCK
                               | CP15_CONTROL_MMU);

    /* Configure the TTB Control register to use only TTB0 */
    CP15TtbCtlTtb0Config();
}

static uint32_t _get_l1_pa_from_va(uint32_t* l1_start, uint32_t va)
{
	uint32_t ret = ((uint32_t)l1_start & 0xFFFFC000);
	ret |= (va & 0xFFF00000) >> 18;
	return ret;
}
static uint32_t _get_l2_pa_from_l1descriptor(uint32_t l1Descriptor, uint32_t va)
{
	uint32_t ret = l1Descriptor & 0xFFFFFC00;
	ret |= (((va >> 12) & 0xFF) << 2);
	return ret;
}
static uint32_t _get_l3_val_from_l2pte(uint32_t l2pte, uint32_t va)
{
	uint32_t ret;
	ret = l2pte & (~0xFFF);
	ret |= (va & 0xFFF);
	return ret;
}

static int32_t MMUMemRegionMapPagesVerify(uint32_t* l1, uint32_t* l2, 
						uint32_t va, tBoolean debug, uint32_t **pLastAddr)
{
	uint32_t addr, entry;
	int32_t status = S_PASS;
	addr = _get_l1_pa_from_va(l1, va);
	entry = (*(uint32_t*)addr);
	addr = _get_l2_pa_from_l1descriptor(entry, va);
	entry = (*(uint32_t*)addr);
	addr = _get_l3_val_from_l2pte(entry, va);
	if(addr != va)
	{
		status = E_INVALID_PARAM;
	}
	if(debug && pLastAddr) *pLastAddr = (uint32_t*)va;
	return status;
}


static int32_t MMUMemRegionMapSmallPagesSecondLevel(
											mmuMemRegionConfig_t *pRegion,
											uint32_t *l2Ptr,
											uint32_t mbIndex,
											tBoolean debug,
											uint32_t **pLastAddress)
{
	uint32_t ptEntry = 0;
	int32_t status = S_PASS, j;
	/* For every 1MB translation in L1, fill L2 of 256 entries */
	for(j = 0;j < 256;j ++)
	{
		if (debug == TRUE && l2Ptr[j] != 0)
		{
			status = E_INVALID_PARAM;
			if(pLastAddress) *pLastAddress = &l2Ptr[j];
			break;
		}

		ptEntry = GENERATE_L2_SMALLPAGE_BASE_ENTRY(pRegion->startAddr, mbIndex, j);
		ptEntry |= MMU_PGTYPE_SMALL_PAGE_L2;

		/* Update the page table entry with Access Permissions. */
		if((S_PASS == status) && (pRegion->accessCtrl > MMU_ACCESS_CTRL_MAX))
		{
			status = E_INVALID_PARAM;
			break;
		}
		ptEntry |= gMmuAccessCtrlForSmallPage[pRegion->accessCtrl];

		/* Inner and outer cache policies - Applicable for normal memory. */
		if((S_PASS == status) &&
			((MMU_MEM_ATTR_NORMAL_SHAREABLE == pRegion->memAttrib) ||
				(MMU_MEM_ATTR_NORMAL_NON_SHAREABLE == pRegion->memAttrib)))
		{
			ptEntry |= (pRegion->innerCachePolicy << 
								MMU_SMALLPAGE_INNER_CACHE_POLICY_SHIFT)|
					   (pRegion->outerCachePolicy << 
								MMU_SMALLPAGE_OUTER_CACHE_POLICY_SHIFT);
		}
		/* Set the memory type (SO, Device, Normal). */
		if((S_PASS == status) && (pRegion->memAttrib <= MMU_MEM_ATTR_MAX))
		{
			  ptEntry |= gMmuMemAttrForSmallPage[pRegion->memAttrib];
		}
		else
		{
			status = E_INVALID_PARAM;
			break;
		}
		l2Ptr[j] = ptEntry;
	}

	return status;
}

int32_t MMUMemRegionMapSmallPages(mmuMemRegionConfig_t *pRegion,
										uint32_t *pMasterPt,
										uint32_t *pLevel2Pt,
										tBoolean debug,
										uint32_t **pLastAddress)
{
    uint32_t *pPageTablePtr;
    uint32_t ptEntry = 0;
    int32_t status = S_PASS;
    int32_t index;

    /* Get the first entry in the page table to set */
    pPageTablePtr = pMasterPt + (pRegion->startAddr >> 
									MMU_PAGEBOUND_SHIFT);

    ptEntry = (MMU_PGTYPE_SMALL_PAGE_L1);

    /* Update the page table entry with security type. */
    if(pRegion->isMemSecure == TRUE)
    {
        ptEntry &= ~MMU_NON_SECURE_MASK_SMALLPAGE;
    }
    else if(pRegion->isMemSecure == FALSE)
    {
        ptEntry |= MMU_NON_SECURE_MASK_SMALLPAGE;
    }
    else
    {
        status = E_INVALID_PARAM;
    }

    if(pMasterPt == NULL)
        status = E_INVALID_PARAM;

    /* Set the entries in the page table for the region attributes */
    if(S_PASS == status)
    {
    	uint32_t maxEntries = (pRegion->numPages >>8);
        for(index = 0; (S_PASS == status) && (index < maxEntries); index ++)
        {
        	uint32_t *val, offset;
			offset = GENERATE_L2_SMALLPAGE_OFFSET(pRegion->startAddr, index);
			val =  pLevel2Pt + offset;
			if (TRUE == debug && pPageTablePtr[index] != 0)
			{
				status = E_INVALID_PARAM;
				if(pLastAddress) *pLastAddress = val;
				break;
			}
			pPageTablePtr[index] = (((uint32_t)val & (~0x1FF)) | ptEntry);
			status = MMUMemRegionMapSmallPagesSecondLevel(
						pRegion, val, index, debug, pLastAddress);
        }
    }
    if(S_PASS == status && TRUE == debug)
    	status = MMUMemRegionMapPagesVerify(pMasterPt, pLevel2Pt, 
							pRegion->startAddr, debug, pLastAddress);

    return status;
}
