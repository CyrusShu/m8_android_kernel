/* g3d/s3c-g3d.c
 *
 * Copyright (c) 2008 Samsung Electronics
 *
 * Samsung S3C G3D driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
 
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/errno.h> 	/* error codes */
#include <asm/div64.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/mman.h>

#include <linux/unistd.h>

#include <linux/version.h>
#include <asm/dma.h>
#include <asm/cacheflush.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>

#include <mach/dma.h>
#include <mach/hardware.h>
#include <mach/map.h>
#include <plat/dma.h>
#include <plat/pm.h>

#include <plat/reserved_mem.h>

#include "s3c_g3d.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28)

#ifdef CONFIG_PLAT_S3C64XX
#include <plat/power-clock-domain.h>

#define S3C_G3D_PA 	(0x72000000)
#define S3C_G3D_IRQ (IRQ_S3C6410_G3D)

#ifdef CONFIG_S3C64XX_DOMAIN_GATING
#define USE_G3D_DOMAIN_GATING
#endif /* CONFIG_S3C64XX_DOMAIN_GATING */

#define DOMAIN_POWER_ON do { \
	s3c_set_normal_cfg(S3C64XX_DOMAIN_G, S3C64XX_ACTIVE_MODE, S3C64XX_3D); \
	if(s3c_wait_blk_pwr_ready(S3C64XX_BLK_G)) { \
		return -1; \
	} \
} while (0)

#define DOMAIN_POWER_OFF do { \
	s3c_set_normal_cfg(S3C64XX_DOMAIN_G, S3C64XX_LP_MODE, S3C64XX_3D); \
} while (0)

#define IS_DOMAIN_POWER_OFF domain_off_check(S3C64XX_DOMAIN_G)

#else
#ifdef CONFIG_PLAT_S5PC1XX
#define S3C_G3D_PA 	(0xEF000000)
#define S3C_G3D_IRQ (IRQ_3D)

#else
#ifdef CONFIG_PLAT_S5P64XX
#include <plat/power_clk_gating.h>

#define S3C_G3D_PA 	(0xD8000000)
#define S3C_G3D_IRQ (IRQ_3D)

#ifdef S5P6442_POWER_GATING_G3D
#define USE_G3D_DOMAIN_GATING
#endif /* S5P6442_POWER_GATING_G3D */

#define DOMAIN_POWER_ON do { \
	s5p6442_pwrgate_config(S5P6442_G3D_ID, S5P6442_ACTIVE_MODE); \
} while (0)

#define DOMAIN_POWER_OFF do { \
	s5p6442_pwrgate_config(S5P6442_G3D_ID, S5P6442_LP_MODE); \
} while (0)

#define IS_DOMAIN_POWER_OFF !s5p6442_blkpower_state(S5P6442_G3D_ID)

#else
#error Unsupported platfotm

#endif /* CONFIG_PLAT_S5P6442 */
#endif /* CONFIG_PLAT_S5PC1XX */
#endif /* CONFIG_PLAT_S3C64XX */

#else

#ifdef CONFIG_PLAT_S3C64XX
#define S3C_G3D_PA 	(0x72000000)
#define S3C_G3D_IRQ (IRQ_3D)

#else
#ifdef CONFIG_PLAT_S5PC1XX
#define S3C_G3D_PA 	(0xEF000000)
#define S3C_G3D_IRQ (IRQ_3D)

#else
#ifdef CONFIG_CPU_S5P6442
// 6442 will be supported from 2.6.29 kernel
#error Unsupported platfotm

#else
#error Unsupported platfotm
#endif /* CONFIG_PLAT_S5P6442 */
#endif /* CONFIG_PLAT_S5PC1XX */
#endif /* CONFIG_PLAT_S3C64XX */

#endif // LINUX_VERSION_CODE >= KERNEL_VERSION(2,8,0)

#define WITH_OPENFIMG

/* G3D driver type */
enum {NONE, FIMG, OPENFIMG};
static int g_driver_type = NONE;
static int g_fops = 0;
struct mutex lock_fops;

static int check_driver_type(int type)
{
	int ret;

	mutex_lock(&lock_fops);
	if (g_driver_type == NONE || g_driver_type == type)
		ret = 1;
	else
		ret = 0;
	mutex_unlock(&lock_fops);

	return ret;
}

typedef uint64_t mem_map_t;
mem_map_t g_uiFreeMemMap = 0x0;
#define MASK1				((mem_map_t)0x1ull)

#define S3C6410_SZ_G3D 		SZ_4K

#define DEBUG_S3C_G3D
#undef	DEBUG_S3C_G3D

#ifdef DEBUG_S3C_G3D
#define DEBUG(fmt,args...) printk(fmt, ##args)
#else
#define DEBUG(fmt,args...) do {} while(0)
#endif

#define G3D_RESERVED_MEM_ADDR_PHY	G3D_RESERVED_START
#define G3D_RESERVED_MEM_SIZE		RESERVED_G3D

#define G3D_CHUNK_SIZE SZ_1M

#define G3D_UI_CHUNK_NUM		(RESERVED_G3D_UI / G3D_CHUNK_SIZE)

static int G3D_CHUNK_NUM = -1; 

typedef struct __alloc_info {
	unsigned int    file_desc_id;
	mem_map_t 	uiAllocedMemMap;
	struct __alloc_info	*next;
} alloc_info;

alloc_info *alloc_info_head = NULL;
alloc_info *alloc_info_tail = NULL;

void g3d_alloc_info_dump(void)
{
	alloc_info *i=alloc_info_head;
	int j;

	printk("g3d_alloc_info_dump\n");
	while(i)
	{
		printk("(ID:0x%x) ",i->file_desc_id);
		for(j=0;j<G3D_CHUNK_NUM;j++)
			if(i->uiAllocedMemMap&(MASK1<<j)) printk("1");
			else printk("0");
		i=i->next;
		if(i) printk(" ->");
	}
	printk("\n");
	if(alloc_info_tail) printk("alloc_info_tail = (ID:0x%x) \n",alloc_info_tail->file_desc_id); 
}

static void* Malloc_3D_ChunkMem(unsigned int szReq, int ithMem)
{	
	unsigned long physicAddr = (G3D_RESERVED_MEM_ADDR_PHY) + (G3D_CHUNK_SIZE * ithMem);
	void * virtAddr = phys_to_virt((unsigned long)physicAddr);

	g_uiFreeMemMap |= (MASK1 << ithMem);
	
	return virtAddr;
}

static void Free_3D_ChunkMem(void* virtAddr,  int ithMem)
{	
	g_uiFreeMemMap &= ~(MASK1 << ithMem);
}

#define G3D_CHUCNK_AVALIABLE	0
#define G3D_CHUCNK_RESERVED		1

#define TIMER_INTERVAL HZ/4

typedef struct {
	int		size;
	unsigned int 	vir_addr;
	unsigned int 	phy_addr;
	unsigned int 	in_used;	/* 0 means avaliable, 1 means reserved */
	unsigned int    file_desc_id;
} s3c_g3d_bootmem;

s3c_g3d_bootmem *g3d_bootm = NULL;// [G3D_CHUNK_NUM];

typedef struct{
	unsigned int pool_buffer_addr;
	unsigned int pool_buffer_size;
	unsigned int hardware_has_single_pipeline;
	unsigned int is_dma_available;
	unsigned int dma_buffer_addr;
	unsigned int dma_buffer_size;
} G3D_CONFIG_STRUCT;

typedef struct{
	unsigned int offset; 	// should be word aligned
	unsigned int size; 	// byte size, should be word aligned
} DMA_BLOCK_STRUCT;

typedef struct {
	ulong src;
	ulong dst;
	int len;
} s3c_3d_dma_info;

#ifdef USE_G3D_DOMAIN_GATING
struct timer_list       g3d_pm_timer;
#endif /* USE_G3D_DOMAIN_GATING */

#define FGGB_PIPESTATE		0x00
#define FGGB_CACHECTL		0x04
#define FGGB_RST		0x08
#define FGGB_VERSION		0x10
#define FGGB_INTPENDING		0x40
#define FGGB_INTMASK		0x44
#define FGGB_PIPEMASK		0x48
#define FGGB_HOSTINTERFACE	0xc000

G3D_CONFIG_STRUCT g3d_config={
#ifdef CONFIG_PLAT_S3C64XX
	S3C_G3D_PA, 	// pool buffer addr
	0x90000, 	// pool_buffer_size
	1, 	// hardware_has_single_pipeline
	0, 	// is_dma_available
#elif CONFIG_PLAT_S5PC1XX
    S3C_G3D_PA,     // pool buffer addr
    0x90000,    // pool_buffer_size
    0,  // hardware_has_single_pipeline
	0,   // is_dma_available
#elif CONFIG_CPU_S5P6442
	S3C_G3D_PA,     // pool buffer addr
	0x90000,    // pool_buffer_size
	1,  // hardware_has_single_pipeline
	0,   // is_dma_available
#else
#error "Hardware did not support 3D funtions."
#endif
	0x57800000,     //dma buffer addr
	0x800000    //dma buffer size
};

#define G3D_IOCTL_MAGIC		'S'
#define WAIT_FOR_FLUSH		_IO(G3D_IOCTL_MAGIC, 100)
#define GET_CONFIG 		_IO(G3D_IOCTL_MAGIC, 101)
//#define START_DMA_BLOCK 	_IO(G3D_IOCTL_MAGIC, 102)

#define S3C_3D_MEM_ALLOC		_IOWR(G3D_IOCTL_MAGIC, 310, struct s3c_3d_mem_alloc)
#define S3C_3D_MEM_FREE			_IOWR(G3D_IOCTL_MAGIC, 311, struct s3c_3d_mem_alloc)
#define S3C_3D_SFR_LOCK			_IO(G3D_IOCTL_MAGIC, 312)
#define S3C_3D_SFR_UNLOCK		_IO(G3D_IOCTL_MAGIC, 313)
#define S3C_3D_MEM_ALLOC_SHARE		_IOWR(G3D_IOCTL_MAGIC, 314, struct s3c_3d_mem_alloc)
#define S3C_3D_MEM_SHARE_FREE		_IOWR(G3D_IOCTL_MAGIC, 315, struct s3c_3d_mem_alloc)
#define S3C_3D_CACHE_INVALID		_IOWR(G3D_IOCTL_MAGIC, 316, struct s3c_3d_mem_alloc)
#define S3C_3D_CACHE_CLEAN			_IOWR(G3D_IOCTL_MAGIC, 317, struct s3c_3d_mem_alloc)
#define S3C_3D_CACHE_CLEAN_INVALID			_IOWR(G3D_IOCTL_MAGIC, 318, struct s3c_3d_mem_alloc)

#define S3C_3D_POWER_INIT 			_IOWR('S', 321, struct s3c_3d_pm_status)
#define S3C_3D_CRITICAL_SECTION  	_IOWR('S', 322, struct s3c_3d_pm_status)
#define S3C_3D_POWER_STATUS 		_IOWR('S', 323, struct s3c_3d_pm_status)


#define MEM_ALLOC		1
#define MEM_ALLOC_SHARE		2

#define PFX 			"s3c_g3d"
#define G3D_MINOR  		249

#define True			1
#define False			0

static wait_queue_head_t waitq;
static struct resource *s3c_g3d_mem;
static void __iomem *s3c_g3d_base;
static int s3c_g3d_irq;
static struct clk *g3d_clock;

static DEFINE_MUTEX(mem_alloc_lock);
static DEFINE_MUTEX(mem_free_lock);
static DEFINE_MUTEX(mem_sfr_lock);

static DEFINE_MUTEX(mem_alloc_share_lock);
static DEFINE_MUTEX(mem_share_free_lock);

static DEFINE_MUTEX(cache_clean_lock);
static DEFINE_MUTEX(cache_invalid_lock);
static DEFINE_MUTEX(cache_clean_invalid_lock);

static DEFINE_MUTEX(pm_power_init);
static DEFINE_MUTEX(pm_critical_section_lock);
static DEFINE_MUTEX(pm_status_lock);

void *dma_3d_done;

struct s3c_3d_mem_alloc {
	int		size;
	unsigned int 	vir_addr;
	unsigned int 	phy_addr;
};

struct s3c_3d_pm_status
{
	unsigned int criticalSection;
	int powerStatus;
	int reserved;
//	int memStatus;
};

static int g_G3D_CriticalFlag = 0;
static int g_G3D_SelfPowerOFF = 0;
static int g_G3D_PowerInit=0;

static unsigned int mutex_lock_processID = 0;

static int flag = 0;

static unsigned int physical_address;

int interrupt_already_recevied;

unsigned int s3c_g3d_base_physical;

void s3c_g3d_release_chunk(unsigned int phy_addr, int size);

///////////// for check memory leak
//*-------------------------------------------------------------------------*/
typedef struct _memalloc_desc
{
	int		size;
	unsigned int 	vir_addr;
	unsigned int 	phy_addr;
	int*    newid;	
	struct _memalloc_desc*  next;
	struct _memalloc_desc*  prev;
} Memalloc_desc;

void garbageCollect(int *newid);

static int g3d_pm_flag = 0;
static void clk_g3d_enable(void)
{
	if (g3d_pm_flag > 0) {
		g3d_pm_flag++;
		return;
	}

	clk_enable(g3d_clock);
	g3d_pm_flag++;
}

static void clk_g3d_disable(void)
{
	if (g3d_pm_flag <= 0) {
		return;
	}

	g3d_pm_flag--;
	clk_disable(g3d_clock);
}

#ifdef USE_G3D_DOMAIN_GATING
static void softReset_g3d(void)
{
    int i=0;
    // device reset
    __raw_writel(1,s3c_g3d_base+FGGB_RST);
    for(i=0;i<1000;i++);
    __raw_writel(0,s3c_g3d_base+FGGB_RST);
    for(i=0;i<1000;i++);

}

void s3c_g3d_timer(void)
{
	if (!g_G3D_PowerInit)
		return;
	if (g_G3D_CriticalFlag > 0)
	{/*turn on*/
		mod_timer(&g3d_pm_timer, jiffies + TIMER_INTERVAL);
		g_G3D_SelfPowerOFF=False;
		return;
	}
	/*turn off*/
	clk_g3d_disable();
	DOMAIN_POWER_OFF;
	g_G3D_SelfPowerOFF=True;
	//printk("[3D] Power off-timer wait\n");    
}
#endif /* USE_G3D_DOMAIN_GATING */

#ifdef WITH_OPENFIMG
static irqreturn_t g3d_handle_irq(int irq, void *dev_id);
#endif
irqreturn_t s3c_g3d_isr(int irq, void *dev_id)
{
#ifdef WITH_OPENFIMG
	if (g_driver_type == OPENFIMG)
		return g3d_handle_irq(irq, dev_id);
#endif
	__raw_writel(0, s3c_g3d_base + FGGB_INTPENDING);

	interrupt_already_recevied = 1;
	wake_up_interruptible(&waitq);

	return IRQ_HANDLED;
}

unsigned int s3c_g3d_get_current_used_mem(void)
{
	int loop_i;
	int iAvalable = 0;
	int iUsed = 0;
	
	for(loop_i = 0; loop_i < G3D_CHUNK_NUM; loop_i++ ){
		if( g3d_bootm[loop_i].in_used == G3D_CHUCNK_AVALIABLE){
			iAvalable++;			
		}
		else { /*G3D_CHUCNK_RESERVED Used Memory*/
			iUsed++;
		}
	}
	printk("used count %d\n", iUsed);

	return iUsed * G3D_CHUNK_SIZE / SZ_1M;
}

void s3c_g3d_dma_finish(struct s3c2410_dma_chan *dma_ch, void *buf_id,
	int size, enum s3c2410_dma_buffresult result){
//	printk("3d dma transfer completed.\n");
	complete(dma_3d_done);
}

int s3c_g3d_open(struct inode *inode, struct file *file)
{
	int *newid;

	if (!check_driver_type(FIMG))
		return -EBUSY;

	newid = (int*)vmalloc(sizeof(int));
	file->private_data = newid;

	g_G3D_SelfPowerOFF=True; //temp first turn on

	/*3D power manager initialized*/
	g_G3D_PowerInit = True;

	mutex_lock(&lock_fops);
	if (++g_fops) {
		printk("use 3D driver type = FIMG, opened = %d\n", g_fops);
		g_driver_type = FIMG;
	}
	mutex_unlock(&lock_fops);

	return 0;
}

int s3c_g3d_release(struct inode *inode, struct file *file)
{
	int *newid = file->private_data;
	if(mutex_lock_processID != 0 && mutex_lock_processID == (unsigned int)file->private_data) {
        	mutex_unlock(&mem_sfr_lock);
	        printk("Abnormal close of pid # %d\n", task_pid_nr(current));        
	}
    
	garbageCollect(newid);
	vfree(newid);

	mutex_lock(&lock_fops);
	if (!(--g_fops)) {
		printk("reset 3D driver type = NONE, opened = %d\n", g_fops);
		g_driver_type = NONE;
	}
	mutex_unlock(&lock_fops);

	return 0;
}

static mem_map_t genMemmapMask(unsigned int uirequestblock)
{
	int i;
	mem_map_t uiMemMask = 0;

	for (i = 0 ; i < uirequestblock ; i++)
{
		uiMemMask |= (MASK1 << i);
	}

	return uiMemMask;
}

void printRemainChunk(void)
{
	int loop_i, j = 0;
		for(loop_i = 0; loop_i < G3D_CHUNK_NUM; loop_i++ ) {
			if( g3d_bootm[loop_i].in_used == G3D_CHUCNK_AVALIABLE ) {
			j++;
			}
		}
	printk("Available Count %d\n", j);
}

static int low_memory_killer(unsigned int uiRequsetBlock, mem_map_t uiMemMask, unsigned int id)
{
	int free_blk = 0;
	alloc_info *s_info = alloc_info_head;
	alloc_info *k_info = NULL;
	unsigned int kill_id;
	int loop_i;

	int chunk_start_num;
	int chunk_end_num;

//	printk("low_memory_killer uiRequsetBlock=%d uiMemMask=0x%x\n",uiRequsetBlock, uiMemMask);
	
	if(!s_info) 
	{
		printk("surfaceflinger's memory not found\n");
		return 0;
	}
	
	chunk_start_num = 0;//G3D_UI_CHUNK_NUM;
	chunk_end_num = G3D_CHUNK_NUM;

	s_info=s_info->next;	// the first s_info is surfaceflinger
	
	while(s_info) 
	{
		for(loop_i = chunk_start_num; loop_i < chunk_end_num - (uiRequsetBlock -1) ; loop_i++ ) {
		
			if ((s_info->uiAllocedMemMap & (uiMemMask << loop_i)) == (uiMemMask << loop_i)){
				k_info=s_info;
				break;
			}
				
		}
		if(k_info) break;
		s_info=s_info->next;
	}

	if(!k_info || k_info->file_desc_id==id) 
	{
		if(k_info==alloc_info_tail) printk("k_info==self\n");
		else printk("k_info==NULL\n");
		printk("oldest 3D process's memory not found\n");
		return 0;
	}

	kill_id = k_info->file_desc_id;
//	printk("low momory killer : kill the oldest process(0x%x)\n",kill_id);

	for(loop_i = chunk_start_num; loop_i < chunk_end_num; loop_i++ ){
		if((g3d_bootm[loop_i].file_desc_id) == (unsigned int)kill_id){
			if (g3d_bootm[loop_i].in_used == G3D_CHUCNK_RESERVED){
		        		s3c_g3d_release_chunk(g3d_bootm[loop_i].phy_addr, g3d_bootm[loop_i].size);
					free_blk++;
			}
			g3d_bootm[loop_i].file_desc_id = 0;
	  	}
  	}

	return free_blk;
}

static unsigned int failed_times = 0;

unsigned long s3c_g3d_available_chunk_size(unsigned int request_size, unsigned int id)
{
	static unsigned int last_failed_id = 0;

	unsigned int loop_i, loop_j;

	unsigned int uiRequsetBlock = (int)(request_size / G3D_CHUNK_SIZE);
	mem_map_t uiMemMask = genMemmapMask(uiRequsetBlock);

	int chunk_start_num;
	int chunk_end_num;
	int is_3d_app;
	int free_blk = 0;

	if (request_size % G3D_CHUNK_SIZE > 0)
		uiRequsetBlock += 1;

	if(!alloc_info_head)
	{
		is_3d_app = 0;
		chunk_start_num = 0;
		chunk_end_num = G3D_CHUNK_NUM;//G3D_UI_CHUNK_NUM;
	}
	else if(alloc_info_head->file_desc_id==id) {
		is_3d_app = 0;
		chunk_start_num = 0;
		chunk_end_num = G3D_CHUNK_NUM;//G3D_UI_CHUNK_NUM;
	}
	else{	
		is_3d_app = 1;
		chunk_start_num = 0;//G3D_UI_CHUNK_NUM;
		chunk_end_num = G3D_CHUNK_NUM;
	}

	for(loop_j = 0; loop_j < 10; loop_j++ ) {
		for(loop_i = chunk_start_num; loop_i < chunk_end_num - (uiRequsetBlock -1) ; loop_i++ ) {
			if ((g_uiFreeMemMap & (uiMemMask << loop_i)) == (uiMemMask << loop_i)) {
				last_failed_id = 0;
				failed_times = 0;
				return G3D_CHUNK_SIZE * uiRequsetBlock;
			}
		}

		if (last_failed_id != id) {
			last_failed_id = id;
			failed_times = 1;
		} else
			failed_times++;

		if (is_3d_app && (failed_times > 1))
			break;

		free_blk = low_memory_killer(uiRequsetBlock, uiMemMask, id);
		if (free_blk > 0)
			continue;

		if (is_3d_app) /* 3D apps retry 1 time, should fix lag problem, though not enough memory. */
			break;

		mutex_unlock(&mem_alloc_lock);
		printk("wait 0.%d sec to get releaing memory\n", loop_j);
		msleep(100);	
		mutex_lock(&mem_alloc_lock);
	}

	if (failed_times == 1)
		printk("s3c_g3d_available_chunk_size failed : %s cannot find adequate memory!", is_3d_app ? "3D apps":"Surfaceflinger");
	else if (failed_times == 2)
		printk("s3c_g3d_available_chunk_size failed : %s cannot find adequate memory! (more failed not print...)\n", is_3d_app ? "3D apps":"Surfaceflinger");
    
	return 0;
}

int check_memStatus(unsigned int id)
{
	alloc_info *s_info = alloc_info_head;

	while(s_info!=NULL)
	{
		if(s_info->file_desc_id == id) return 0;
		s_info = s_info->next;
	}

	return 1;
}

void register_alloc_info(int index, mem_map_t uiAllocedMemMap)
{
	unsigned int id = g3d_bootm[index].file_desc_id;
	alloc_info *s_info = alloc_info_head;

//	printk("<register_alloc_info index=%d id=0x%x AllocedMemMap=0x%x>\n",index,id,uiAllocedMemMap);
	while(s_info!=NULL)
	{
		if(s_info->file_desc_id == id) break;
		s_info = s_info->next;
	}
	
	if(s_info)
	{
		if(s_info->uiAllocedMemMap & uiAllocedMemMap) 
				printk("err uiAllocedMemMap\n");
		else s_info->uiAllocedMemMap|=uiAllocedMemMap;
	}
	else
	{
		s_info = vmalloc(sizeof(alloc_info));	
		s_info->file_desc_id 	= id;
		s_info->next 			= NULL;
		s_info->uiAllocedMemMap	= uiAllocedMemMap;

		if(alloc_info_tail)	
			alloc_info_tail->next = s_info;
		else	
			alloc_info_head = s_info;
		alloc_info_tail = s_info;

	}
#ifdef DEBUG_S3C_G3D
	g3d_alloc_info_dump();
#endif

}

unsigned long s3c_g3d_reserve_chunk(struct file* filp, unsigned int size)
{
	unsigned int loop_i, loop_j;

	unsigned int uiRequsetBlock = (int)(size / G3D_CHUNK_SIZE);
	mem_map_t uiMemMask = genMemmapMask(uiRequsetBlock);

	int chunk_start_num;
	int chunk_end_num;

	unsigned int id = (unsigned int)filp->private_data;

	if (size % G3D_CHUNK_SIZE > 0)
		printk("s3c_g3d_reserve_chunk : wrong estimated reserve memory\n");

	if(!alloc_info_head)
	{
		chunk_start_num = 0;
		chunk_end_num = G3D_CHUNK_NUM;//G3D_UI_CHUNK_NUM;
	}
	else if(alloc_info_head->file_desc_id==id) {
		chunk_start_num = 0;
		chunk_end_num = G3D_CHUNK_NUM;//G3D_UI_CHUNK_NUM;
	}
	else{	
		chunk_start_num = 0;//G3D_UI_CHUNK_NUM;
		chunk_end_num = G3D_CHUNK_NUM;
	}

 	for(loop_i = chunk_start_num; loop_i < chunk_end_num - (uiRequsetBlock -1); loop_i++ ) {

		if ((g_uiFreeMemMap & (uiMemMask << loop_i)) == (uiMemMask << loop_i)) // check free memory at memory map
		{
			for(loop_j = loop_i; loop_j < loop_i + uiRequsetBlock ; loop_j++ )
			{
				g3d_bootm[loop_j].in_used = G3D_CHUCNK_RESERVED;
				g3d_bootm[loop_j].file_desc_id = (int)filp->private_data;
		 	}

			g_uiFreeMemMap &= ~(uiMemMask << loop_i); // remove free chunk block at memory map
			register_alloc_info(loop_i,uiMemMask << loop_i);

#ifdef DEBUG_S3C_G3D
			printRemainChunk();
#endif
			return g3d_bootm[loop_i].phy_addr;
		}
	}

	printk("s3c_g3d_reserve_chunk failed : Cannot find adequate memory!\n");
    
	return 0;
}

void unregister_alloc_info(int index, mem_map_t uiAllocedMemMap)
{
	unsigned int id = g3d_bootm[index].file_desc_id;
	alloc_info *s_info = alloc_info_head;
	alloc_info *pre_info = NULL;

//	printk("<unregister_alloc_info index=%d id=0x%x>\n",index,id);
	
	while(s_info!=NULL)
	{
		if(s_info->file_desc_id == id) break;
		pre_info = s_info;
		s_info = s_info->next;
	}
	
	if(s_info)
	{
		if(s_info->uiAllocedMemMap==uiAllocedMemMap)
		{
			if(pre_info)
			{
				pre_info->next = s_info->next;
				if(alloc_info_tail==s_info) alloc_info_tail = pre_info;
			}
			else	// case : surfaceflinger is killed 
			{
//				printk("warnning : surfaceflinger is killed\n");
				alloc_info_head = s_info->next;
				if(alloc_info_tail==s_info) alloc_info_tail = NULL;
			}

			vfree(s_info);
		}
		else
		{
			if ((s_info->uiAllocedMemMap & uiAllocedMemMap) != uiAllocedMemMap) {
				printk("unregister_alloc_info err \n");
				return;
			}
			s_info->uiAllocedMemMap &= ~uiAllocedMemMap;
		}
	}
	else
	{
		printk("unregister_alloc_info err \n");
		return;
	}
#ifdef DEBUG_S3C_G3D
	g3d_alloc_info_dump();
#endif
	
}

void s3c_g3d_release_chunk(unsigned int phy_addr, int size)
{
	unsigned int loop_i, loop_j;
	struct mm_struct *mm = current->mm;

	unsigned int uiRequsetBlock = (int)(size / G3D_CHUNK_SIZE);
	mem_map_t uiMemMask = genMemmapMask(uiRequsetBlock);

	for(loop_i = 0; loop_i < G3D_CHUNK_NUM - (uiRequsetBlock - 1); loop_i++ ) {
		if( g3d_bootm[loop_i].phy_addr == phy_addr ) {

			if ((g_uiFreeMemMap & (uiMemMask << loop_i)) != 0) // check free memory at memory map
			{
				printk("s3c_g3d_release_chunk : memory map is crashed");
				break;
			}
	
			do_munmap(mm, g3d_bootm[loop_i].vir_addr, size);
			g_uiFreeMemMap |= (uiMemMask << loop_i); // add free chunk block at memory map
			unregister_alloc_info(loop_i,uiMemMask << loop_i);

			for(loop_j = loop_i; loop_j < loop_i + uiRequsetBlock ; loop_j++ )
			{
				g3d_bootm[loop_j].in_used = G3D_CHUCNK_AVALIABLE;
				g3d_bootm[loop_j].file_desc_id = 0;
			}
	
			break;
		}
	}

	if(loop_i >= G3D_CHUNK_NUM)
		printk("s3c_g3d_release_chunk failed : Cannot find the phys_addr : 0x%p\n", (void*)phy_addr);

#ifdef DEBUG_S3C_G3D
	printRemainChunk();	
#endif
}

static int s3c_g3d_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	u32 val;
	DECLARE_COMPLETION_ONSTACK(complete);

	struct mm_struct *mm = current->mm;
	struct s3c_3d_mem_alloc param;
	struct s3c_3d_pm_status param_pm;

	unsigned int timer;
	
	switch (cmd) {
	case WAIT_FOR_FLUSH:
		//if fifo has already been flushed, return;
		val = __raw_readl(s3c_g3d_base+FGGB_PIPESTATE);
		//printk("read pipestate = 0x%x\n",val);
		if((val & arg) ==0) break;

		// enable interrupt
		interrupt_already_recevied = 0;
		__raw_writel(0x0001171f,s3c_g3d_base+FGGB_PIPEMASK);
		__raw_writel(1,s3c_g3d_base+FGGB_INTMASK);

		//printk("wait for flush (arg=0x%lx)\n",arg);

		timer = 1000000;

		while(timer) {
			wait_event_interruptible_timeout(waitq, (interrupt_already_recevied>0), 1*HZ);

			__raw_writel(0,s3c_g3d_base+FGGB_INTMASK);
			interrupt_already_recevied = 0;
			//if(interrupt_already_recevied==0)interruptible_sleep_on(&waitq);
			val = __raw_readl(s3c_g3d_base+FGGB_PIPESTATE);
			//printk("in while read pipestate = 0x%x\n",val);
			if(val & arg){
			} else{
				break;
			}
			__raw_writel(1,s3c_g3d_base+FGGB_INTMASK);
			timer --;
		}
		break;

	case GET_CONFIG:
		if (copy_to_user((void *)arg,&g3d_config,sizeof(G3D_CONFIG_STRUCT))) {
			printk("G3D: copy_to_user failed to get g3d_config\n");
			return -EFAULT;		
		}
		break;

#if 0
	case START_DMA_BLOCK:
		if (copy_from_user(&dma_block,(void *)arg,sizeof(DMA_BLOCK_STRUCT))) {
			printk("G3D: copy_to_user failed to get dma_block\n");
			return -EFAULT;		
		}

		if (dma_block.offset%4!=0) {
			printk("G3D: dma offset is not aligned by word\n");
			return -EINVAL;
		}
		if (dma_block.size%4!=0) {
			printk("G3D: dma size is not aligned by word\n");
			return -EINVAL;
		}
		if (dma_block.offset+dma_block.size >g3d_config.dma_buffer_size) {
			printk("G3D: offset+size exceeds dam buffer\n");
			return -EINVAL;
		}

		dma_info.src = g3d_config.dma_buffer_addr+dma_block.offset;
		dma_info.len = dma_block.size;
		dma_info.dst = s3c_g3d_base_physical+FGGB_HOSTINTERFACE;

		DEBUG(" dma src=0x%x\n", dma_info.src);
		DEBUG(" dma len =%u\n", dma_info.len);
		DEBUG(" dma dst = 0x%x\n", dma_info.dst);

		dma_3d_done = &complete;

		if (s3c2410_dma_request(DMACH_3D_M2M, &s3c6410_3d_dma_client, NULL)) {
			printk(KERN_WARNING "Unable to get DMA channel(DMACH_3D_M2M).\n");
			return -EFAULT;
		}

		s3c2410_dma_set_buffdone_fn(DMACH_3D_M2M, s3c_g3d_dma_finish);
		s3c2410_dma_devconfig(DMACH_3D_M2M, S3C_DMA_MEM2MEM, 1, (u_long) dma_info.src);
		s3c2410_dma_config(DMACH_3D_M2M, 4, 4);
		s3c2410_dma_setflags(DMACH_3D_M2M, S3C2410_DMAF_AUTOSTART);

		//consistent_sync((void *) dma_info.dst, dma_info.len, DMA_FROM_DEVICE);
	//	s3c2410_dma_enqueue(DMACH_3D_M2M, NULL, (dma_addr_t) virt_to_dma(NULL, dma_info.dst), dma_info.len);
		s3c2410_dma_enqueue(DMACH_3D_M2M, NULL, (dma_addr_t) dma_info.dst, dma_info.len);

	//	printk("wait for end of dma operation\n");
		wait_for_completion(&complete);
	//	printk("dma operation is performed\n");

		s3c2410_dma_free(DMACH_3D_M2M, &s3c6410_3d_dma_client);

		break;
#endif

	case S3C_3D_MEM_ALLOC:		
		mutex_lock(&mem_alloc_lock);
		if(copy_from_user(&param, (struct s3c_3d_mem_alloc *)arg, sizeof(struct s3c_3d_mem_alloc))){
			mutex_unlock(&mem_alloc_lock);			
			return -EFAULT;
		}
       
		flag = MEM_ALLOC;
		
		param.size = s3c_g3d_available_chunk_size(param.size,(unsigned int)file->private_data);

		if (param.size == 0){
			if (failed_times == 1)
				printk("S3C_3D_MEM_ALLOC FAILED because there is no block memory bigger than you request\n");
			else if (failed_times == 2)
				printk("S3C_3D_MEM_ALLOC FAILED because there is no block memory bigger than you request (more failed not print...)\n");
			flag = 0;
			mutex_unlock(&mem_alloc_lock);			
			return -EFAULT;
		}			
             
		param.vir_addr = do_mmap(file, 0, param.size, PROT_READ|PROT_WRITE, MAP_SHARED, 0);
		DEBUG("param.vir_addr = %08x\n", param.vir_addr);

		if(param.vir_addr == -EINVAL) {
			printk("S3C_3D_MEM_ALLOC FAILED\n");
			flag = 0;
			mutex_unlock(&mem_alloc_lock);			
			return -EFAULT;
		}
		param.phy_addr = physical_address;

       // printk("alloc %d\n", param.size);
		DEBUG("KERNEL MALLOC : param.phy_addr = 0x%X \t size = %d \t param.vir_addr = 0x%X\n", param.phy_addr, param.size, param.vir_addr);

		if(copy_to_user((struct s3c_3d_mem_alloc *)arg, &param, sizeof(struct s3c_3d_mem_alloc))){
			flag = 0;
			mutex_unlock(&mem_alloc_lock);
			return -EFAULT;		
		}

		flag = 0;
		
//		printk("\n\n====Success the malloc from kernel=====\n");
		mutex_unlock(&mem_alloc_lock);
		
		break;

	case S3C_3D_MEM_FREE:	
		mutex_lock(&mem_free_lock);
		if(copy_from_user(&param, (struct s3c_3d_mem_alloc *)arg, sizeof(struct s3c_3d_mem_alloc))){
			mutex_unlock(&mem_free_lock);
			return -EFAULT;
		}

		DEBUG("KERNEL FREE : param.phy_addr = 0x%X \t size = %d \t param.vir_addr = 0x%X\n", param.phy_addr, param.size, param.vir_addr);

		/*
		if (do_munmap(mm, param.vir_addr, param.size) < 0) {
			printk("do_munmap() failed !!\n");
			mutex_unlock(&mem_free_lock);
			return -EINVAL;
		}
		*/

		s3c_g3d_release_chunk(param.phy_addr, param.size);
		//printk("KERNEL : virt_addr = 0x%X\n", virt_addr);
		//printk("free %d\n", param.size);


		param.size = 0;
		DEBUG("do_munmap() succeed !!\n");

		if(copy_to_user((struct s3c_3d_mem_alloc *)arg, &param, sizeof(struct s3c_3d_mem_alloc))){
			mutex_unlock(&mem_free_lock);
			return -EFAULT;
		}
		
		mutex_unlock(&mem_free_lock);
		
		break;

	case S3C_3D_SFR_LOCK:
		mutex_lock(&mem_sfr_lock);
		mutex_lock_processID = (unsigned int)file->private_data;
		//DEBUG("s3c_g3d_ioctl() : You got a muxtex lock !!\n");
		break;

	case S3C_3D_SFR_UNLOCK:
		mutex_lock_processID = 0;
		mutex_unlock(&mem_sfr_lock);
		//DEBUG("s3c_g3d_ioctl() : The muxtex unlock called !!\n");
		break;

	case S3C_3D_MEM_ALLOC_SHARE:		
		mutex_lock(&mem_alloc_share_lock);
		if(copy_from_user(&param, (struct s3c_3d_mem_alloc *)arg, sizeof(struct s3c_3d_mem_alloc))){
			mutex_unlock(&mem_alloc_share_lock);
			return -EFAULT;
		}
		flag = MEM_ALLOC_SHARE;

		physical_address = param.phy_addr;

		DEBUG("param.phy_addr = %08x\n", physical_address);

		param.vir_addr = do_mmap(file, 0, param.size, PROT_READ|PROT_WRITE, MAP_SHARED, 0);
		DEBUG("param.vir_addr = %08x\n", param.vir_addr);

		if(param.vir_addr == -EINVAL) {
			printk("S3C_3D_MEM_ALLOC_SHARE FAILED\n");
			flag = 0;
			mutex_unlock(&mem_alloc_share_lock);
			return -EFAULT;
		}

		DEBUG("MALLOC_SHARE : param.phy_addr = 0x%X \t size = %d \t param.vir_addr = 0x%X\n", param.phy_addr, param.size, param.vir_addr);

		if(copy_to_user((struct s3c_3d_mem_alloc *)arg, &param, sizeof(struct s3c_3d_mem_alloc))){
			flag = 0;
			mutex_unlock(&mem_alloc_share_lock);
			return -EFAULT;		
		}

		flag = 0;
		
		mutex_unlock(&mem_alloc_share_lock);
		
		break;

	case S3C_3D_MEM_SHARE_FREE:	
		mutex_lock(&mem_share_free_lock);
		if(copy_from_user(&param, (struct s3c_3d_mem_alloc *)arg, sizeof(struct s3c_3d_mem_alloc))){
			mutex_unlock(&mem_share_free_lock);
			return -EFAULT;		
		}

		DEBUG("MEM_SHARE_FREE : param.phy_addr = 0x%X \t size = %d \t param.vir_addr = 0x%X\n", param.phy_addr, param.size, param.vir_addr);

		if (do_munmap(mm, param.vir_addr, param.size) < 0) {
			printk("do_munmap() failed - MEM_SHARE_FREE!!\n");
			mutex_unlock(&mem_share_free_lock);
			return -EINVAL;
		}

		param.vir_addr = 0;
		DEBUG("do_munmap() succeed !! - MEM_SHARE_FREE\n");

		if(copy_to_user((struct s3c_3d_mem_alloc *)arg, &param, sizeof(struct s3c_3d_mem_alloc))){
			mutex_unlock(&mem_share_free_lock);
			return -EFAULT;		
		}

		mutex_unlock(&mem_share_free_lock);
		
		break;

	case S3C_3D_CACHE_INVALID:
		mutex_lock(&cache_invalid_lock);
		if(copy_from_user(&param, (struct s3c_3d_mem_alloc *)arg, sizeof(struct s3c_3d_mem_alloc))){
			printk("ERR: Invalid Cache Error\n");	
			mutex_unlock(&cache_invalid_lock);
			return -EFAULT;	
		}
		dmac_inv_range((const void *) param.vir_addr,(const void *)(param.vir_addr + param.size));
		mutex_unlock(&cache_invalid_lock);
		break;

	case S3C_3D_CACHE_CLEAN:
		mutex_lock(&cache_clean_lock);
		if(copy_from_user(&param, (struct s3c_3d_mem_alloc *)arg, sizeof(struct s3c_3d_mem_alloc))){
			printk("ERR: Invalid Cache Error\n");	
			mutex_unlock(&cache_clean_lock);
			return -EFAULT;	
		}
		dmac_clean_range((const void *) param.vir_addr,(const void *)(param.vir_addr + param.size));
		mutex_unlock(&cache_clean_lock);
		break;

	case S3C_3D_CACHE_CLEAN_INVALID:
		mutex_lock(&cache_clean_invalid_lock);
		if(copy_from_user(&param, (struct s3c_3d_mem_alloc *)arg, sizeof(struct s3c_3d_mem_alloc))){
			mutex_unlock(&cache_clean_invalid_lock);
			printk("ERR: Invalid Cache Error\n");	
			return -EFAULT;	
		}
		dmac_flush_range((const void *) param.vir_addr,(const void *)(param.vir_addr + param.size));
		mutex_unlock(&cache_clean_invalid_lock);
		break;

	case S3C_3D_POWER_INIT:
		if(copy_from_user(&param_pm, (struct s3c_3d_pm_status *)arg, sizeof(struct s3c_3d_pm_status))){
			printk("ERR: Invalid Cache Error\n");	
			return -EFAULT;	
		}
		break;

	case S3C_3D_CRITICAL_SECTION:
#ifdef USE_G3D_DOMAIN_GATING
		mutex_lock(&pm_critical_section_lock);
		if(copy_from_user(&param_pm, (struct s3c_3d_pm_status *)arg, sizeof(struct s3c_3d_pm_status))){
			printk("ERR: Invalid Cache Error\n");	
			mutex_unlock(&pm_critical_section_lock);
			return -EFAULT;	
		}

//		param_pm.memStatus = check_memStatus((unsigned int)file->private_data);

		if(param_pm.criticalSection) g_G3D_CriticalFlag++;
		else g_G3D_CriticalFlag--;

		if(g_G3D_CriticalFlag==0)
		{/*kick power off*/
			/*power off*/
			/*kick timer*/
			mod_timer(&g3d_pm_timer, jiffies + TIMER_INTERVAL);
		}
		else if(g_G3D_CriticalFlag>0)
		{/*kick power on*/
			if(IS_DOMAIN_POWER_OFF)
			{/*if powered off*/                        
				if(g_G3D_SelfPowerOFF)
				{/*powered off by 3D PM or by Resume*/
					/*power on*/
					DOMAIN_POWER_ON;
					clk_g3d_enable();
					/*Need here??*/
					softReset_g3d();
					// printk("[3D] Power on\n");  
				}
				else
				{
					/*powered off by the system :: error*/
					printk("Error on the system :: app tries to work during sleep\n");
					mutex_unlock(&pm_critical_section_lock);
					return -EFAULT;	
				}
			}
			else
			{
				/*already powered on : nothing to do*/
				//g_G3D_SelfPowerOFF=0;
			}
		}
		else if(g_G3D_CriticalFlag < 0) 
		{
			printk("Error on the system :: g_G3D_CriticalFlag < 0\n");
		}
//		printk("S3C_3D_CRITICAL_SECTION: param_pm.criticalSection=%d\n",param_pm.criticalSection);

		if (copy_to_user((void *)arg,&param_pm,sizeof(struct s3c_3d_pm_status)))
		{
			printk("G3D: copy_to_user failed to get s3c_3d_pm_status\n");

			mutex_unlock(&pm_critical_section_lock);
			return -EFAULT;		
		}
		mutex_unlock(&pm_critical_section_lock);
#endif /* USE_G3D_DOMAIN_GATING */
		break;

	default:
		DEBUG("s3c_g3d_ioctl() : default !!\n");
		return -EINVAL;
	}
	
	return 0;
}

int s3c_g3d_mmap(struct file* filp, struct vm_area_struct *vma)
{
	unsigned long pageFrameNo, size, phys_addr;

	size = vma->vm_end - vma->vm_start;

	switch (flag) { 
	case MEM_ALLOC :
		phys_addr = s3c_g3d_reserve_chunk(filp, size);

		if (phys_addr == 0) {
			printk("There is no reserved memory for G3D!\n");
			return -EINVAL;
		}

//		DEBUG("MMAP_ALLOC : virt addr = 0x%p, size = %d\n", virt_addr, size);
//		printk("MMAP_ALLOC : virt addr = 0x%p, phys addr = 0x%p, size = %d\n", (void*)virt_addr, (void*)phys_addr, (int)(size));
		physical_address = (unsigned int)phys_addr;

		pageFrameNo = __phys_to_pfn(phys_addr);
		break;
		
	case MEM_ALLOC_SHARE :
//		DEBUG("MMAP_KMALLOC_SHARE : phys addr = 0x%p\n", physical_address);
		
		// page frame number of the address for the physical_address to be shared.
		pageFrameNo = __phys_to_pfn(physical_address);
		//DEBUG("MMAP_KMALLOC_SHARE: PFN = 0x%x\n", pageFrameNo);
//		DEBUG("MMAP_KMALLOC_SHARE : vma->end = 0x%p, vma->start = 0x%p, size = %d\n", vma->vm_end, vma->vm_start, size);
		break;
		
	default :
		printk("here\n");

		// page frame number of the address for a source G2D_SFR_SIZE to be stored at.
		pageFrameNo = __phys_to_pfn(S3C_G3D_PA);
//		DEBUG("MMAP : vma->end = 0x%p, vma->start = 0x%p, size = %d\n", vma->vm_end, vma->vm_start, size);

		if(size > S3C6410_SZ_G3D) {
			printk("The size of G3D_SFR_SIZE mapping is too big!\n");
			return -EINVAL;
		}
		break;
	}
	
//	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if ((vma->vm_flags & VM_WRITE) && !(vma->vm_flags & VM_SHARED)) {
		printk("s3c_g3d_mmap() : Writable G3D_SFR_SIZE mapping must be shared !\n");
		return -EINVAL;
	}

	if (remap_pfn_range(vma, vma->vm_start, pageFrameNo, size, vma->vm_page_prot)) {
		printk("s3c_g3d_mmap() : remap_pfn_range() failed !\n");
		return -EINVAL;
	}

	return 0;
}

void garbageCollect(int* newid)
{
	int loop_i;

//      printk("====Garbage Collect is executed====\n");
        mutex_lock(&mem_alloc_lock);

	for(loop_i = 0; loop_i < G3D_CHUNK_NUM; loop_i++ ){
		if((g3d_bootm[loop_i].file_desc_id) == (unsigned int)newid){
				if (g3d_bootm[loop_i].in_used == G3D_CHUCNK_RESERVED){
		        		s3c_g3d_release_chunk(g3d_bootm[loop_i].phy_addr, g3d_bootm[loop_i].size);
				}
				g3d_bootm[loop_i].file_desc_id = 0;
	        }
       }
        mutex_unlock(&mem_alloc_lock);  
}

#ifdef WITH_OPENFIMG
/*
 * OpenFIMG Kernel Interface
 *
 * Copyright 2010-2011 Tomasz Figa <tomasz.figa at gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
/*
 * Various definitions
 */
#define G3D_AUTOSUSPEND_DELAY		(1000)
#define G3D_TIMEOUT			(1*HZ)

/*
 * Registers
 */
#define G3D_FGGB_PIPESTAT_REG		(0x00)
#define G3D_FGGB_PIPESTAT_MSK		(0x0005171f)

#define G3D_FGGB_CACHECTL_REG		(0x04)
#define G3D_FGGB_FLUSH_MSK		(0x00000033)
#define G3D_FGGB_INVAL_MSK		(0x00001300)

#define G3D_FGGB_RESET_REG		(0x08)
#define G3D_FGGB_VERSION		(0x10)
#define G3D_FGGB_INTPENDING_REG		(0x40)
#define G3D_FGGB_INTMASK_REG		(0x44)
#define G3D_FGGB_PIPEMASK_REG		(0x48)
#define G3D_FGGB_PIPETGTSTATE_REG	(0x4c)

/*
 * Private structures
 */
struct g3d_context;

struct g3d_drvdata {
	void __iomem		*base;

	uint32_t		mask;
	struct mutex		lock;
	struct mutex		hw_lock;
	struct g3d_context	*hw_owner;
	struct completion	completion;

	int			irq;
	struct resource 	*mem;
	struct clk		*clock;
	struct device		*dev;
	struct miscdevice	mdev;
};

static struct g3d_drvdata *drvdata;

struct g3d_context {
	struct g3d_drvdata	*data;
	/* More to come */
};

/*
 * Register accessors
 */
static inline void g3d_write(struct g3d_drvdata *d, uint32_t b, uint32_t r)
{
	writel(b, d->base + r);
}

static inline uint32_t g3d_read(struct g3d_drvdata *d, uint32_t r)
{
	return readl(d->base + r);
}

/*
 * Hardware operations
 */
static inline void g3d_soft_reset(struct g3d_drvdata *data)
{
	g3d_write(data, 1, G3D_FGGB_RESET_REG);
	udelay(1);
	g3d_write(data, 0, G3D_FGGB_RESET_REG);
}

static inline int g3d_flush_pipeline(struct g3d_drvdata *data, unsigned int mask)
{
	int ret = 0;

	if((g3d_read(data, G3D_FGGB_PIPESTAT_REG) & mask) == 0)
		return 0;

	/* Setup the interrupt */
	data->mask = mask;
	init_completion(&data->completion);
	g3d_write(data, 0, G3D_FGGB_PIPEMASK_REG);
	g3d_write(data, 0, G3D_FGGB_PIPETGTSTATE_REG);
	g3d_write(data, mask, G3D_FGGB_PIPEMASK_REG);
	g3d_write(data, 1, G3D_FGGB_INTMASK_REG);

	/* Check if the condition isn't already met */
	if((g3d_read(data, G3D_FGGB_PIPESTAT_REG) & mask) == 0) {
		/* Disable the interrupt */
		g3d_write(data, 0, G3D_FGGB_INTMASK_REG);
		return 0;
	}

	if(!wait_for_completion_interruptible_timeout(&data->completion,
								G3D_TIMEOUT)) {
		dev_err(data->dev, "timeout while waiting for interrupt, resetting (stat=%08x)\n",
					g3d_read(data, G3D_FGGB_PIPESTAT_REG));
		g3d_soft_reset(data);
		ret = -EFAULT;
	}

	/* Disable the interrupt */
	g3d_write(data, 0, G3D_FGGB_INTMASK_REG);

	return ret;
}

static inline void g3d_flush_caches(struct g3d_drvdata *data)
{
	int timeout = 1000000000;
	g3d_write(data, G3D_FGGB_FLUSH_MSK, G3D_FGGB_CACHECTL_REG);

	do {
		if(!g3d_read(data, G3D_FGGB_CACHECTL_REG))
			break;
	} while (--timeout);
}

static inline void g3d_invalidate_caches(struct g3d_drvdata *data)
{
	int timeout = 1000000000;
	g3d_write(data, G3D_FGGB_INVAL_MSK, G3D_FGGB_CACHECTL_REG);

	do {
		if(!g3d_read(data, G3D_FGGB_CACHECTL_REG))
			break;
	} while (--timeout);
}

/*
 * State processing
 */
static irqreturn_t g3d_handle_irq(int irq, void *dev_id)
{
#if 0
	struct g3d_drvdata *data = (struct g3d_drvdata *)dev_id;
#endif
	struct g3d_drvdata *data = platform_get_drvdata((struct platform_device *)dev_id);
	uint32_t stat;

	g3d_write(data, 0, G3D_FGGB_INTPENDING_REG);
	stat = g3d_read(data, G3D_FGGB_PIPESTAT_REG) & data->mask;

	if(!stat)
		complete(&data->completion);

	return IRQ_HANDLED;
}

static inline int ctx_has_lock(struct g3d_context *ctx)
{
	struct g3d_drvdata *data = ctx->data;

	return mutex_is_locked(&data->hw_lock) && (data->hw_owner == ctx);
}

/*
 * File operations
 */
static int s3c_g3d_unlock(struct g3d_context *ctx)
{
	struct g3d_drvdata *data = ctx->data;
	int ret = 0;

	mutex_lock(&data->lock);

	if (unlikely(!ctx_has_lock(ctx))) {
		dev_err(data->dev, "called S3C_G3D_UNLOCK without holding the hardware lock\n");
		ret = -EPERM;
		goto exit;
	}

	mutex_unlock(&data->hw_lock);

	dev_dbg(data->dev, "hardware lock released by %p\n", ctx);

exit:
	mutex_unlock(&data->lock);

	return ret;
}

static int s3c_g3d_lock(struct g3d_context *ctx)
{
	struct g3d_drvdata *data = ctx->data;
	int ret = 0;

	mutex_lock(&data->hw_lock);

	dev_dbg(data->dev, "hardware lock acquired by %p\n", ctx);

	mutex_lock(&data->lock);

	if (likely(data->hw_owner == ctx)) {
		mutex_unlock(&data->lock);
		return 0;
	}

	ret = 1;

	if (data->hw_owner) {
		g3d_flush_pipeline(data, G3D_FGGB_PIPESTAT_MSK);
		ret = 2;
	}

	data->hw_owner = ctx;

exit:
	mutex_unlock(&data->lock);

	return ret;
}

static int s3c_g3d_flush(struct g3d_context *ctx, u32 mask)
{
	struct g3d_drvdata *data = ctx->data;
	int ret = 0;

	mutex_lock(&data->lock);

	if (unlikely(!ctx_has_lock(ctx))) {
		dev_err(data->dev, "called S3C_G3D_FLUSH without holding the hardware lock\n");
		ret = -EPERM;
		goto exit;
	}

	ret = g3d_flush_pipeline(data, mask & G3D_FGGB_PIPESTAT_MSK);

exit:
	mutex_unlock(&data->lock);

	return ret;
}

static int s3c_g3d2_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct g3d_context *ctx = file->private_data;
	int ret = 0;

	switch(cmd) {
	/* Prepare and lock the hardware */
	case S3C_G3D_LOCK:
		ret = s3c_g3d_lock(ctx);
		break;

	/* Unlock the hardware and start idle timer */
	case S3C_G3D_UNLOCK:
		ret = s3c_g3d_unlock(ctx);
		break;

	/* Wait for the hardware to finish its work */
	case S3C_G3D_FLUSH:
		ret = s3c_g3d_flush(ctx, arg & G3D_FGGB_PIPESTAT_MSK);
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static int s3c_g3d2_open(struct inode *inode, struct file *file)
{
	struct g3d_context *ctx;

	if (!check_driver_type(OPENFIMG))
		return -EBUSY;

	ctx = kmalloc(sizeof(struct g3d_context), GFP_KERNEL);
	ctx->data = drvdata;

	file->private_data = ctx;

	mutex_lock(&lock_fops);
	if (++g_fops) {
		printk("use 3D driver type = OPENFIMG, opened = %d\n", g_fops);
		g_driver_type = OPENFIMG;
	}
	mutex_unlock(&lock_fops);

	dev_dbg(ctx->data->dev, "device opened\n");

	return 0;
}

static int s3c_g3d2_release(struct inode *inode, struct file *file)
{
	struct g3d_context *ctx = file->private_data;
	struct g3d_drvdata *data = ctx->data;
	int unlock = 0;

	/* Do this atomically */
	mutex_lock(&data->lock);

	unlock = ctx_has_lock(ctx);

	mutex_unlock(&data->lock);

	/* Unlock if we have the lock */
	if(unlock)
		s3c_g3d2_ioctl(inode, file, S3C_G3D_UNLOCK, 0);

	kfree(ctx);

	mutex_lock(&lock_fops);
	if (!(--g_fops)) {
		printk("reset 3D driver type = NONE, opened = %d\n", g_fops);
		g_driver_type = NONE;
	}
	mutex_unlock(&lock_fops);

	dev_dbg(data->dev, "device released\n");

	return 0;
}

int s3c_g3d2_mmap(struct file* file, struct vm_area_struct *vma)
{
	struct g3d_context *ctx = file->private_data;
	struct g3d_drvdata *data = ctx->data;
	unsigned long pfn;
	size_t size = vma->vm_end - vma->vm_start;

	pfn = __phys_to_pfn(data->mem->start);

	if(size > resource_size(data->mem)) {
		dev_err(data->dev, "mmap size bigger than G3D SFR block\n");
		return -EINVAL;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if ((vma->vm_flags & VM_WRITE) && !(vma->vm_flags & VM_SHARED)) {
		dev_err(data->dev, "mmap of G3D SFR block must be shared\n");
		return -EINVAL;
	}

	if (remap_pfn_range(vma, vma->vm_start, pfn, size, vma->vm_page_prot)) {
		dev_err(data->dev, "remap_pfn range failed\n");
		return -EINVAL;
	}

	dev_dbg(data->dev, "hardware mapped by %p\n", ctx);

	return 0;
}

static int s3c_g3d2_suspend(struct platform_device *dev)
{
	struct g3d_drvdata *data = platform_get_drvdata(dev);

	if (mutex_is_locked(&data->hw_lock)) {
		dev_err(&dev->dev, "suspend requested with locked hardware (broken userspace?)\n");
		return -EAGAIN;
	}

	clk_disable(data->clock);
	data->hw_owner = NULL;

	return 0;
}

static int s3c_g3d2_resume(struct platform_device *dev)
{
	struct g3d_drvdata *data = platform_get_drvdata(dev);

	clk_enable(data->clock);
	g3d_soft_reset(data);

	return 0;
}
#endif

static struct file_operations s3c_g3d_fops = {
	.owner 	= THIS_MODULE,
	.ioctl 	= s3c_g3d_ioctl,
	.open 	= s3c_g3d_open,
	.release = s3c_g3d_release,
	.mmap	= s3c_g3d_mmap,
};

#ifdef WITH_OPENFIMG
static struct file_operations s3c_g3d2_fops = {
	.owner		= THIS_MODULE,
	.ioctl		= s3c_g3d2_ioctl,
	.open		= s3c_g3d2_open,
	.release	= s3c_g3d2_release,
	.mmap		= s3c_g3d2_mmap,
};
#endif

static struct miscdevice s3c_g3d_dev = {
	.minor		= G3D_MINOR,
	.name		= "s3c-g3d",
	.fops		= &s3c_g3d_fops,
};

int s3c_g3d_probe(struct platform_device *pdev)
{
	struct resource *res;
#ifdef WITH_OPENFIMG
	struct g3d_drvdata *data;
#endif
	int		ret;
	int		size;
	int		i, loop_i;
	uint32_t        version;

	DEBUG("s3c_g3d probe() called\n");

#ifdef USE_G3D_DOMAIN_GATING
	DOMAIN_POWER_ON;
#endif /* USE_G3D_DOMAIN_GATING */

	g3d_clock = clk_get(&pdev->dev, "hclk_g3d");
	if(g3d_clock == NULL) {
		printk(KERN_ERR PFX "failed to find post clock source\n");
		ret = -ENOENT;
		goto err_clock;
	}

	clk_g3d_enable();

	/* get the memory region for the post processor driver */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(res == NULL) {
		printk(KERN_ERR PFX "failed to get memory region resouce\n");
		ret = -ENOENT;
		goto err_mem;
	}

	s3c_g3d_base_physical = (unsigned int)res->start;

	size = (res->end-res->start)+1;
	s3c_g3d_mem = request_mem_region(res->start, size, pdev->name);
	if(s3c_g3d_mem == NULL) {
		printk(KERN_ERR PFX "failed to reserve memory region\n");
		ret = -ENOENT;
		goto err_mem;
	}
	
	s3c_g3d_base = ioremap(res->start, size);
	if(s3c_g3d_base == NULL) {
		printk(KERN_ERR PFX "failed ioremap\n");
		ret = -ENOENT;
		goto err_ioremap;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if(res == NULL) {
		printk(KERN_ERR PFX "failed to get irq resource\n");
		ret = -ENOENT;
		goto err_irq;
	}
	
	s3c_g3d_irq = res->start;
	ret = request_irq(res->start, s3c_g3d_isr, 0, pdev->name, pdev);
	if (ret != 0) {
		printk(KERN_ERR PFX "failed to install irq (%d)\n", ret);
		goto err_irq;
	}

	init_waitqueue_head(&waitq);

	ret = misc_register(&s3c_g3d_dev);
	if (ret < 0) {
		printk (KERN_ERR "cannot register miscdev on minor=%d (%d)\n",
				G3D_MINOR, ret);
		goto err_misc_register;
	}

#ifdef USE_G3D_DOMAIN_GATING
	/* init pm timer */
	init_timer(&g3d_pm_timer);
	g3d_pm_timer.function = (void*) s3c_g3d_timer;
#endif /* USE_G3D_DOMAIN_GATING */

	/* device reset */
	__raw_writel(1,s3c_g3d_base+FGGB_RST);
	for(i=0;i<1000;i++);
	__raw_writel(0,s3c_g3d_base+FGGB_RST);
	for(i=0;i<1000;i++);

	version = __raw_readl(s3c_g3d_base + FGGB_VERSION);

	G3D_CHUNK_NUM = G3D_RESERVED_MEM_SIZE / G3D_CHUNK_SIZE;

	if (g3d_bootm == NULL)
		g3d_bootm = kmalloc(sizeof(s3c_g3d_bootmem) * G3D_CHUNK_NUM, GFP_KERNEL);	

	printk("s3c_g3d version : 0x%x\n", version);
	printk("G3D_RESERVED_MEM_SIZE : %d MB\n", G3D_RESERVED_MEM_SIZE/SZ_1M);
	printk("G3D_CHUNK_SIZE : %d MB\n", G3D_CHUNK_SIZE/SZ_1M);
	printk("G3D_CHUNK_NUM : %d (UI_CHUNK:%d)\n", G3D_CHUNK_NUM, G3D_UI_CHUNK_NUM);

	for( loop_i = 0; loop_i < G3D_CHUNK_NUM; loop_i++ ){
		g3d_bootm[loop_i].vir_addr = (unsigned int)Malloc_3D_ChunkMem(G3D_CHUNK_SIZE, loop_i);
		g3d_bootm[loop_i].phy_addr = (unsigned int)virt_to_phys((void*)g3d_bootm[loop_i].vir_addr);
		g3d_bootm[loop_i].in_used = G3D_CHUCNK_AVALIABLE;
		g3d_bootm[loop_i].size = G3D_CHUNK_SIZE;
		g3d_bootm[loop_i].file_desc_id = 0;

		printk("%d th virt_addr = 0x%p, phy_addr = 0x%p\n",
			(int)loop_i, (void*)(g3d_bootm[loop_i].vir_addr), (void*)(g3d_bootm[loop_i].phy_addr));
	}

	mutex_init(&lock_fops);

#ifdef WITH_OPENFIMG
	printk("with OpenFIMG 3D driver by Tomasz Figa <tomasz.figa at gmail.com>\n");
	data = kzalloc(sizeof(struct g3d_drvdata), GFP_KERNEL);
	if (data == NULL) {
		dev_err(&pdev->dev, "failed to allocate driver data.\n");
			goto err_with_openfimg;
	}

	/* initialize the miscdevice struct */
	data->mdev.minor	= MISC_DYNAMIC_MINOR;
	data->mdev.name		= "s3c-g3d2";
	data->mdev.fops		= &s3c_g3d2_fops;

	/* get device clock */
	data->clock = g3d_clock;
	/* reserve the memory */
	data->mem = s3c_g3d_mem;
	/* map the memory */
	data->base = s3c_g3d_base;
	/* get the IRQ */
	/* request the IRQ */
	data->irq = s3c_g3d_irq;

	data->dev = &pdev->dev;
	data->hw_owner = NULL;
	mutex_init(&data->lock);
	mutex_init(&data->hw_lock);
	init_completion(&data->completion);

	platform_set_drvdata(pdev, data);
	drvdata = data;

	printk("detected FIMG-3DSE version %d.%d.%d\n",
		version >> 24, (version >> 16) & 0xff, (version >> 8) & 0xff);

	ret = misc_register(&data->mdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "could not register miscdev (%d)\n", ret);
		goto err_misc_register;
	}
#endif

	/* check to see if everything is setup correctly */
	return 0;

#ifdef WITH_OPENFIMG
err_with_openfimg:
#endif
err_misc_register:
	free_irq(res->start, pdev);
err_irq:
	iounmap(s3c_g3d_base);
err_ioremap:
        release_resource(s3c_g3d_mem);
        kfree(s3c_g3d_mem);
	s3c_g3d_mem = NULL;
err_mem:
	clk_g3d_enable();
err_clock:

#ifdef USE_G3D_DOMAIN_GATING
	DOMAIN_POWER_OFF;
#endif /* USE_G3D_DOMAIN_GATING */

	return ret;
}

static int s3c_g3d_suspend(struct platform_device *dev, pm_message_t state)
{
#ifdef WITH_OPENFIMG
	if (g_driver_type == OPENFIMG)
		return s3c_g3d2_suspend(dev);
#endif

	if(g_G3D_CriticalFlag)
	{
		printk("unexpected Suspend : App don't support suspend-mode.\n");
	}
	else
	{
		/*power off*/
	
		clk_g3d_disable();

#ifdef USE_G3D_DOMAIN_GATING
		DOMAIN_POWER_OFF;
#endif /* USE_G3D_DOMAIN_GATING */

		g_G3D_CriticalFlag=0;
		g_G3D_SelfPowerOFF=False;
	}
    
	return 0;
}

static int s3c_g3d_remove(struct platform_device *dev)
{
#ifdef WITH_OPENFIMG
	struct g3d_drvdata *data = platform_get_drvdata(dev);
#endif

	free_irq(s3c_g3d_irq, NULL);

	if (s3c_g3d_mem != NULL) {
		pr_debug("s3c_g3d: releasing s3c_post_mem\n");
		iounmap(s3c_g3d_base);
		release_resource(s3c_g3d_mem);
		kfree(s3c_g3d_mem);
	}

	if (g3d_bootm != NULL)
		kfree(g3d_bootm);

	misc_deregister(&s3c_g3d_dev);
#ifdef WITH_OPENFIMG
	misc_deregister(&data->mdev);
#endif

	clk_g3d_disable();

#ifdef USE_G3D_DOMAIN_GATING
	DOMAIN_POWER_OFF;
#endif /* USE_G3D_DOMAIN_GATING */

	return 0;
}

static int s3c_g3d_resume(struct platform_device *pdev)
{
#ifdef WITH_OPENFIMG
	if (g_driver_type == OPENFIMG)
		return s3c_g3d2_resume(pdev);
#endif
	if(!g_G3D_CriticalFlag)
	{
		/*power on 3D PM right after 3D APIs are used*/
		g_G3D_SelfPowerOFF=True;
	}

	return 0;
}

static struct platform_driver s3c_g3d_driver = {
	.probe          = s3c_g3d_probe,
	.remove         = s3c_g3d_remove,
	.suspend        = s3c_g3d_suspend,
	.resume         = s3c_g3d_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "s3c-g3d",
	},
};

static char banner[] __initdata = KERN_INFO "S3C G3D Driver, (c) 2007-2009 Samsung Electronics\n";

int __init  s3c_g3d_init(void)
{

	printk(banner);
	if(platform_driver_register(&s3c_g3d_driver)!=0)
	{
		printk("platform device register Failed \n");
		return -1;
	}

	return 0;
}

void  s3c_g3d_exit(void)
{
    int loop_i;
	platform_driver_unregister(&s3c_g3d_driver);

	for( loop_i = 0; loop_i < G3D_CHUNK_NUM; loop_i++ ){
		Free_3D_ChunkMem((void*)g3d_bootm[loop_i].vir_addr, loop_i);
		
		g3d_bootm[loop_i].vir_addr = 0;
		g3d_bootm[loop_i].phy_addr = 0;
		g3d_bootm[loop_i].in_used = G3D_CHUCNK_AVALIABLE;
		g3d_bootm[loop_i].file_desc_id = 0;
		g3d_bootm[loop_i].size = 0;

//		printk("%d th virt_addr = 0x%p, phy_addr = 0x%p\n", loop_i, (void*)(g3d_bootm[loop_i].vir_addr), (void*)(g3d_bootm[loop_i].phy_addr));
	}

	printk("S3C G3D module exit\n");
}

module_init(s3c_g3d_init);
module_exit(s3c_g3d_exit);

MODULE_AUTHOR("lee@samsung.com");
MODULE_DESCRIPTION("S3C G3D Device Driver");
MODULE_LICENSE("GPL");


