/******************************************************************************
 * Copyright (c) 2013-2016 Realtek Semiconductor Corp.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/
#include <osdep_service.h>
#include <stdio.h>
#include "rtl8721d_cache.h"
/********************* os depended utilities ********************/

#ifndef USE_MUTEX_FOR_SPINLOCK
#define USE_MUTEX_FOR_SPINLOCK 1
#endif

#define USE_PTHREAD_MUTEX 0 //todo

//----- ------------------------------------------------------------------
// Misc Function
//----- ------------------------------------------------------------------
extern void timer_wrapper(_timerHandle timer_hdl);

static irqstate_t tizen_flags;
void save_and_cli()
{
	tizen_flags = 0;
	tizen_flags = irqsave();
}

void restore_flags()
{
	irqrestore(tizen_flags);
}

void cli()
{
}

/* Not needed on 64bit architectures */
static unsigned int __div64_32(u64 *n, unsigned int base)
{
	u64 rem = *n;
	u64 b = base;
	u64 res, d = 1;
	unsigned int high = rem >> 32;

	/* Reduce the thing a bit first */
	res = 0;
	if (high >= base) {
		high /= base;
		res = (u64)high << 32;
		rem -= (u64)(high * base) << 32;
	}

	while ((u64)b > 0 && b < rem) {
		b = b + b;
		d = d + d;
	}

	do {
		if (rem >= b) {
			rem -= b;
			res += d;
		}
		b >>= 1;
		d >>= 1;
	} while (d);

	*n = res;
	return rem;
}

/********************* os depended service ********************/

u8 *_tizenrt_malloc(u32 sz)
{
	return kmm_zalloc(sz);
}
u8 *_tizenrt_zmalloc(u32 sz)
{
	u8 *pbuf = _tizenrt_malloc(sz);

	if (pbuf != NULL)
		memset(pbuf, 0, sz);

	return pbuf;
}

void _tizenrt_mfree(u8 *pbuf, u32 sz)
{
	/* To avoid gcc warnings */
	( void ) sz;

	kmm_free(pbuf);
}

static void _tizenrt_memcpy(void *dst, void *src, u32 sz)
{
	memcpy(dst, src, sz);
}

static int _tizenrt_memcmp(void *dst, void *src, u32 sz)
{
	if (!(memcmp(dst, src, sz)))
		return 1;

	return 0;
}

static void _tizenrt_memset(void *pbuf, int c, u32 sz)

{
	memset(pbuf, c, sz);
}

static void _tizenrt_init_sema(_sema *sema, int init_val)
{
	if (*sema == NULL) {
		*sema = (_sema)_tizenrt_zmalloc(sizeof(sem_t));
		if (*sema == NULL) {
			DiagPrintf("\r\n Failed to kmm_zalloc in %s\n", __FUNCTION__);
			return;
		}
	} else {
		DiagPrintf("\r\n already inited %s\n", __FUNCTION__);
		return;
	}
	int ret = sem_init((sem_t *)(*sema), 0, init_val);
	if (ret != OK) {
			DiagPrintf("\r\n Failed to sem_init in %s\n", __FUNCTION__);
			_tizenrt_mfree(*sema, sizeof(sem_t));
			return;
	}
}

static void _tizenrt_free_sema(_sema *sema)
{
	int i;
	if (*sema != NULL) {
		i = sem_destroy(*sema);
		if (i == 0) {
			kmm_free(*sema);
		} else {
			DiagPrintf("\r\n _tizenrt_free_sema fail!!! \n");
		}
	}
	*sema = NULL;
}

static void _tizenrt_up_sema(_sema *sema)
{
	sem_post((sem_t *)(*sema));
}

static void _tizenrt_up_sema_from_isr(_sema *sema)
{
	sem_post((sem_t *)(*sema));
}

static u32 _tizenrt_down_sema(_sema *sema, u32 timeout)
{
	struct timespec ts;
	int ret;
	if(timeout == RTW_MAX_DELAY){
		ret = sem_wait((sem_t *)(*sema));
	} else {
		clock_gettime(CLOCK_REALTIME, &ts);
		ts.tv_sec += timeout / 1000;
		ret = sem_timedwait((sem_t *)(*sema), &ts);
	}
	if (ret != OK) {
		return _FAIL;
	} else {
		return _SUCCESS;
	}
}

static void _tizenrt_mutex_init(_mutex *pmutex)
{
#if USE_PTHREAD_MUTEX
	pthread_mutexattr_t mutex_attr;
	int err;

	if (*pmutex == NULL) {
		*pmutex = _tizenrt_zmalloc(sizeof(pthread_mutex_t));
		if (*pmutex == NULL) {
			ndbg("\r\n _tizenrt_mutex_init failed \n");
			goto err_exit;
		}
	}
	err = pthread_mutexattr_init(&mutex_attr);
	if (err) {
		ndbg("pthread_mutexattr_init failed with error code (%d).", err);
		goto err_exit;
	}
	err = pthread_mutexattr_settype(&mutex_attr, PTHREAD_MUTEX_RECURSIVE);
	if (err) {
		ndbg("pthread_mutexattr_settype failed with error code (%d).", err);
		goto err_exit;
	}
	err = pthread_mutex_init((pthread_mutex_t *)(*pmutex), &mutex_attr);
	if (err) {
		ndbg("pthread_mutex_init failed with error code (%d).", err);
		goto err_exit;
	}
	return;
err_exit:
	if(*pmutex)
		_tizenrt_mfree(*pmutex);
	return;
#else
	if (*pmutex == NULL) {
		*pmutex = _tizenrt_zmalloc(sizeof(sem_t));
		if (*pmutex == NULL) {
			ndbg("\r\n _tizenrt_mutex_init failed \n");
			return;
		}
	}
	sem_init(*pmutex, 0, 1);
	sem_setprotocol(*pmutex, SEM_PRIO_NONE);
#endif
}

static void _tizenrt_mutex_free(_mutex *pmutex)
{
#if USE_PTHREAD_MUTEX
	if(*pmutex == NULL)
		return;
	int err = pthread_mutex_destroy((pthread_mutex_t *)(*pmutex));
	if (err) {
		ndbg("pthread_mutex_destroy failed with error code (%d).", err);
	}
	_tizenrt_mfree(*pmutex);
#else
	int i;
	if (*pmutex != NULL) {
		i = sem_destroy(*pmutex);
		if (i == 0) {
			kmm_free(*pmutex);
		} else {
			ndbg("\r\n _tizenrt_mutex_free fail!!! \n");
		}
	}
#endif
	*pmutex = NULL;
}

static void _tizenrt_mutex_get(_mutex *plock)
{
#if USE_PTHREAD_MUTEX
	int err = pthread_mutex_lock((pthread_mutex_t *)(*plock));
	if (err) {
		ndbg("Failed to acquire lock. Error code: (%d).", err);
	}
#else
	int temp;
	temp = sem_wait(*plock);
	if (temp != 0) {
		DBG_ERR("_tizenrt_spinlock failed!\n");
	}
#endif
}

static int _tizenrt_mutex_get_timeout(_mutex *plock, u32 timeout_ms)
{
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	ts.tv_sec += timeout_ms / 1000;
	if (sem_timedwait(*plock, &ts) < 0) {
		return _FAIL;
	} else {
		return _SUCCESS;
	}
}

static void _tizenrt_mutex_put(_lock *plock)
{
	sem_post(*plock);
}

static void _tizenrt_enter_critical(_lock *plock, _irqL *pirqL)
{
	save_and_cli();
}

static void _tizenrt_exit_critical(_lock *plock, _irqL *pirqL)
{
	restore_flags();
}

static void _tizenrt_enter_critical_from_isr(_lock *plock, _irqL *pirqL)
{
	DiagPrintf("%s %d\r\n", __func__, __LINE__);
}

static void _tizenrt_exit_critical_from_isr(_lock *plock, _irqL *pirqL)
{
	DiagPrintf("%s %d\r\n", __func__, __LINE__);
}

static int _tizenrt_enter_critical_mutex(_mutex *pmutex, _irqL *pirqL)
{
	int temp;
	temp = sem_wait(*pmutex);
	if (temp != 0) {
		DBG_ERR("_tizenrt_spinlock failed!\n");
	}
	return temp;
}

static void _tizenrt_exit_critical_mutex(_mutex *pmutex, _irqL *pirqL)

{
	sem_post(*pmutex);
}

#if defined(CONFIG_PLATFORM_8195BHP)
#include "timer_api.h"
static gtimer_t tmp_timer_obj;
#endif
static void _tizenrt_cpu_lock(void)
{
#if defined(CONFIG_PLATFORM_8195BHP)
	__disable_irq();
	icache_disable();
	dcache_disable();

	gtimer_init(&tmp_timer_obj, 0xff);
	gtimer_reload(&tmp_timer_obj, 400 * 1000); // 4s
	gtimer_start(&tmp_timer_obj);
#endif
}

static void _tizenrt_cpu_unlock(void)
{
#if defined(CONFIG_PLATFORM_8195BHP)
	int duration = (int)gtimer_read_us(&tmp_timer_obj) / 1000;
	gtimer_deinit(&tmp_timer_obj);
	// compensate rtos tick
	vTaskIncTick(duration);
	icache_enable();
	icache_invalidate();
	__enable_irq();
#endif
}

static void _tizenrt_spinlock_init(_lock *plock)
{
	if (*plock == NULL) {
		*plock = _tizenrt_zmalloc(sizeof(sem_t));
		if (*plock == NULL) {
			ndbg("\r\n _tizenrt_spinlock_init failed \n");
			return;
		}
	}
	sem_init(*plock, 0, 1);
	sem_setprotocol(*plock, SEM_PRIO_NONE);
}

static void _tizenrt_spinlock_free(_lock *plock)
{
	int i;
	if (*plock != NULL) {
		i = sem_destroy(*plock);
		if (i == 0) {
			kmm_free(*plock);
		} else {
			ndbg("\r\n _tizenrt_spinlock_free fail!!! \n");
		}
	}
	*plock = NULL;
}

static void _tizenrt_spinlock(_lock *plock)

{
	int temp;
	temp = sem_wait(*plock);
	if (temp != 0) {
		DBG_ERR("_tizenrt_spinlock failed!\n");
	}
}

static void _tizenrt_spinunlock(_lock *plock)
{
	sem_post(*plock);
}

static void _tizenrt_spinlock_irqsave(_lock *plock, _irqL *irqL)
{

	int temp;
	__enable_irq();
	temp = pthread_mutex_lock(*plock);
	if (temp != 0) {
		DBG_ERR("_tizenrt_spinlock failed!\n");
	}
}

static void _tizenrt_spinunlock_irqsave(_lock *plock, _irqL *irqL)
{

	pthread_mutex_unlock(*plock);
	__disable_irq();
}

static int _tizenrt_init_xqueue(_xqueue *queue, const char *name, u32 message_size, u32 number_of_messages)
{
	DiagPrintf("%s %d\r\n", __func__, __LINE__);
	return 0;
}

static int _tizenrt_push_to_xqueue(_xqueue *queue, void *message, u32 timeout_ms)
{
	DiagPrintf("%s %d\r\n", __func__, __LINE__);
	return 0;
}

static int _tizenrt_pop_from_xqueue(_xqueue *queue, void *message, u32 timeout_ms)
{
	DiagPrintf("%s %d\r\n", __func__, __LINE__);
	return 0;
}

static int _tizenrt_deinit_xqueue(_xqueue *queue)
{
	DiagPrintf("%s %d\r\n", __func__, __LINE__);
	return 0;
}

static u32 _tizenrt_get_current_time(void)
{
	return (u32)clock();
}

static u32 _tizenrt_systime_to_ms(u32 sys_time)
{

	return (sys_time * 1000L / TICK_PER_SEC);
}

static u32 _tizenrt_systime_to_sec(u32 sys_time)
{

	return (sys_time / TICK_PER_SEC);
}

static u32 _tizenrt_ms_to_systime(u32 ms)
{

	return (ms * TICK_PER_SEC / 1000L);
}

static u32 _tizenrt_sec_to_systime(u32 sec)
{

	return (sec * TICK_PER_SEC);
}

static void _tizenrt_msleep_os(int ms)
{
	usleep((unsigned int)ms * 1000);
}

static void _tizenrt_usleep_os(int us)
{
	usleep((unsigned int)us);
}

static void _tizenrt_mdelay_os(int ms)
{
	up_mdelay((unsigned long)ms);
}

static void _tizenrt_udelay_os(int us)
{
	up_udelay((unsigned long)us);
}

static void _tizenrt_yield_os(void)
{
	sched_yield();
}

static void _tizenrt_ATOMIC_SET(ATOMIC_T *v, int i)
{
	atomic_set(v, i);
}

static int _tizenrt_ATOMIC_READ(ATOMIC_T *v)
{
	return atomic_read(v);
}

static void _tizenrt_ATOMIC_ADD(ATOMIC_T *v, int i)
{
	save_and_cli();
	v->counter += i;
	restore_flags();
}

static void _tizenrt_ATOMIC_SUB(ATOMIC_T *v, int i)
{
	save_and_cli();
	v->counter -= i;
	restore_flags();
}

static void _tizenrt_ATOMIC_INC(ATOMIC_T *v)
{
	_tizenrt_ATOMIC_ADD(v, 1);
}

static void _tizenrt_ATOMIC_DEC(ATOMIC_T *v)
{
	_tizenrt_ATOMIC_SUB(v, 1);
}

static int _tizenrt_ATOMIC_ADD_RETURN(ATOMIC_T *v, int i)
{
	int temp;

	save_and_cli();
	temp = v->counter;
	temp += i;
	v->counter = temp;
	restore_flags();

	return temp;
}

static int _tizenrt_ATOMIC_SUB_RETURN(ATOMIC_T *v, int i)
{
	int temp;

	save_and_cli();
	temp = v->counter;
	temp -= i;
	v->counter = temp;
	restore_flags();

	return temp;
}

static int _tizenrt_ATOMIC_INC_RETURN(ATOMIC_T *v)
{
	return _tizenrt_ATOMIC_ADD_RETURN(v, 1);
}

static int _tizenrt_ATOMIC_DEC_RETURN(ATOMIC_T *v)
{
	return _tizenrt_ATOMIC_SUB_RETURN(v, 1);
}

static u64 _tizenrt_modular64(u64 n, u64 base)
{
	unsigned int __base = (base);
	unsigned int __rem;

	if (((n) >> 32) == 0) {
		__rem = (unsigned int)(n) % __base;
		(n) = (unsigned int)(n) / __base;
	} else {
		__rem = __div64_32(&(n), __base);
	}

	return __rem;
}

/* Refer to ecos bsd tcpip codes */
static int _tizenrt_arc4random(void)
{
#if defined(CONFIG_PLATFORM_8721D)
	int value = (int)Rand();
	return value;

#else
	u32 res = xTaskGetTickCount();
	static unsigned long seed = 0xDEADB00B;

#if defined(CONFIG_PLATFORM_8711B)
	if(random_seed){
		seed = random_seed;
		random_seed = 0;
	}
#endif

	seed = ((seed & 0x007F00FF) << 7) ^
		((seed & 0x0F80FF00) >> 8) ^ // be sure to stir those low bits
		(res << 13) ^ (res >> 9);	// using the clock too!
	return (int)seed;
#endif
}

static int _tizenrt_get_random_bytes(void *buf, size_t len)
{
	unsigned int ranbuf;
	unsigned int *lp;
	int i, count;
	count = len / sizeof(unsigned int);
	lp = (unsigned int *)buf;

	for (i = 0; i < count; i++) {
		lp[i] = _tizenrt_arc4random();
		len -= sizeof(unsigned int);
	}

	if (len > 0) {
		ranbuf = _tizenrt_arc4random();
		_tizenrt_memcpy(&lp[i], &ranbuf, len);
	}
	return 0;
}

static u32 _tizenrt_GetFreeHeapSize(void)

{
	return 0;
}
void *tcm_heap_malloc(int size);
extern int debug_steps;
static int _tizenrt_create_task(struct task_struct *ptask, const char *name,
		u32 stack_size, u32 priority, thread_func_t func, void *thctx)
{
	pthread_attr_t attr;
	struct sched_param sparam;
	int res = 0;

	pthread_t *tid = _tizenrt_zmalloc(sizeof(pthread_t));
	if (tid == NULL) {
		DBG_ERR("Failed to malloc for tid\n");
		goto err_exit;
	}

	res = pthread_attr_init(&attr);
	if (res != OK) {
		DBG_ERR("Failed to pthread_attr_init\n");
		goto err_exit;
	}

	stack_size *= sizeof(uint32_t);
	res = pthread_attr_setstacksize(&attr, stack_size);
	if (res != OK) {
		DBG_ERR("Failed to pthread_attr_setstacksize\n");
		goto err_exit;
	}
	
	sparam.sched_priority = PTHREAD_DEFAULT_PRIORITY + priority;
	res = pthread_attr_setschedparam(&attr, &sparam);
	if (res != OK) {
		DBG_ERR("Failed to pthread_attr_setstacksize\n");
		goto err_exit;
	}

	res = pthread_create(tid, &attr, (pthread_startroutine_t)func, thctx);
	if (res != OK) {
		DBG_ERR("Failed to pthread_create\n");
		goto err_exit;
	}
	if (*tid == 0) {
		DBG_ERR("create the task %s failed!", name);
		goto err_exit;
	}
	ptask->task = tid;
	ptask->task_name = name;
	return _SUCCESS;
err_exit:
	if (tid) {
		_tizenrt_mfree(tid, sizeof(*tid));
	}
	ptask->task = NULL;
	ptask->task_name = NULL;
	return _FAIL;
}

static void _tizenrt_delete_task(struct task_struct *ptask)
{
	int status = 0;
	pthread_t *tid = (pthread_t *)ptask->task;
	if (!ptask->task || *tid == 0) {
		DBG_ERR("_tizenrt_delete_task(): ptask is NULL %s!\n", ptask->task_name);
		return;
	}
	//status = task_delete(ptask->task); //TODO
	status = pthread_cancel(*tid);
	if (status != OK) {
		DBG_ERR("delete the task failed!", ptask->task_name);
		return;
	}
	_tizenrt_mfree(tid, sizeof(*tid));
	ptask->task = NULL;
	return;
}

void _tizenrt_wake_task(struct task_struct *task)
{
	sem_post(task->wakeup_sema);
	return;
}

static void _tizenrt_thread_enter(char *name)
{
	DBG_INFO("\n\rRTKTHREAD %s\n", name);
}

static void _tizenrt_thread_exit(void)
{
	DBG_INFO("\n\rRTKTHREAD exit %s\n", __FUNCTION__);
	pthread_exit(NULL);
	//task_delete(0);
}

_timerHandle _tizenrt_timerCreate(const signed char *pcTimerName,
		osdepTickType xTimerPeriodInTicks,
		u32 uxAutoReload,
		void * pvTimerID, 
		TIMER_FUN pxCallbackFunction )
{
	struct timer_list_priv *timer = (struct timer_list_priv *)_tizenrt_zmalloc(sizeof(struct timer_list_priv));
	if (timer == NULL) {
		DBG_ERR("Fail to alloc priv");
		//rtw_timerDelete(timer, TIMER_MAX_DELAY);
		//timer->timer_hdl = NULL;
		return NULL;
	}
	timer->work_hdl = (struct work_s *)_tizenrt_zmalloc(sizeof(struct work_s));
	if (timer->work_hdl == NULL) {
		DBG_ERR("Fail to alloc timer->work_hdl");
		//_tizenrt_timerDelete(timer, TIMER_MAX_DELAY);
		//timer->timer_hdl = NULL;
		kmm_free(timer);
		return NULL;
	}
	timer->live = 0;
	timer->timevalue = xTimerPeriodInTicks;
	timer->data = pvTimerID;
	timer->function = pxCallbackFunction;
	return (_timerHandle)timer;
}

u32 _tizenrt_timerDelete(_timerHandle xTimer,
		osdepTickType xBlockTime)
{
	struct timer_list_priv *timer = (struct timer_list_priv *)xTimer;
	int ret = work_cancel(LPWORK, timer->work_hdl);
	if (ret != OK) {
		goto cleanup;
	}
	timer->data = NULL;
	timer->timer_hdl = NULL;
	timer->timevalue = 0;
	timer->live = 0;
	kmm_free(timer->work_hdl);
	kmm_free(timer);
	return _SUCCESS;

cleanup:
	if (ret != -2) {
		timer->data = NULL;
		timer->timer_hdl = NULL;
		timer->timevalue = 0;
		timer->live = 0;
		kmm_free(timer->work_hdl);
		kmm_free(timer);
		DBG_ERR("_tizenrt_del_timer failed! ret = %d", ret);
		return _FAIL;
	}
	timer->data = NULL;
	timer->timer_hdl = NULL;
	timer->timevalue = 0;
	timer->live = 0;
	kmm_free(timer->work_hdl);
	kmm_free(timer);
	//DBG_ERR("_tizenrt_del_timer is Done! timer->work_hdl = %x", timer->work_hdl);
	return _SUCCESS;
}

u32 _tizenrt_timerIsTimerActive(_timerHandle xTimer)
{
	struct timer_list_priv *timer = (struct timer_list_priv *)xTimer;

	return timer->live;
	//return (timer->work_hdl->worker != NULL);
}

u32 _tizenrt_timerStop(_timerHandle xTimer,
		osdepTickType xBlockTime)
{
	struct timer_list_priv *timer = (struct timer_list_priv *)xTimer;

	int ret = work_cancel(LPWORK, timer->work_hdl);
	if (ret != OK) {
		goto cleanup;
	}
	timer->timevalue = 0;
	timer->live = 0;
	return _SUCCESS;

cleanup:
	if (ret != -2) {
		kmm_free(timer->work_hdl);
		kmm_free(timer);
		DBG_ERR("_tizenrt_stop_timer failed! ret = %d", ret);
		return _FAIL;
	}
	timer->timevalue = 0;
	timer->live = 0;
	DBG_ERR("_tizenrt_stop_timer is Done! timer->work_hdl = %x", timer->work_hdl);
	return _SUCCESS;
}

u32 _tizenrt_timerChangePeriod( _timerHandle xTimer,
							   osdepTickType xNewPeriod,
							   osdepTickType xBlockTime)
{
	int ret;
	struct timer_list_priv *timer = (struct timer_list_priv *)xTimer;
	ret = work_queue(LPWORK, timer->work_hdl, timer->function, (void *)(timer), xNewPeriod);
	if (ret == -EALREADY) {
		if (work_cancel(LPWORK, timer->work_hdl) != OK) {
			goto cleanup;
		}
		if (work_queue(LPWORK, timer->work_hdl, timer->function, (void *)(timer), xNewPeriod)) {
			goto cleanup;
		}
	}
	return _SUCCESS;

cleanup:
	kmm_free(timer->work_hdl);
	timer->timer_hdl = NULL;
	timer->live = 0;
	kmm_free(timer);
	DBG_ERR("_tizenrt_set_timer failed!");
	return _FAIL;
}

void *_tizenrt_timerGetID(_timerHandle xTimer)
{
	struct timer_list_priv *timer = (struct timer_list_priv *)xTimer;
	return timer->data;
}

u32 _tizenrt_timerStart(_timerHandle xTimer,
		osdepTickType xBlockTime)
{
	DiagPrintf("%s %d\r\n", __func__, __LINE__);
	return 0;
}

u32 _tizenrt_timerStartFromISR(_timerHandle xTimer,
		osdepBASE_TYPE *pxHigherPriorityTaskWoken)
{
	DiagPrintf("%s %d\r\n", __func__, __LINE__);
	return 0;
}

u32 _tizenrt_timerStopFromISR(_timerHandle xTimer,
		osdepBASE_TYPE *pxHigherPriorityTaskWoken)
{
	DiagPrintf("%s %d\r\n", __func__, __LINE__);
	return 0;
}

u32 _tizenrt_timerResetFromISR(_timerHandle xTimer,
							   osdepBASE_TYPE *pxHigherPriorityTaskWoken)
{
	DiagPrintf("%s %d\r\n", __func__, __LINE__);
	return 0;
}

u32 _tizenrt_timerChangePeriodFromISR(_timerHandle xTimer,
									  osdepTickType xNewPeriod,
									  osdepBASE_TYPE *pxHigherPriorityTaskWoken)
{
	DiagPrintf("%s %d\r\n", __func__, __LINE__);
	return 0;
}

u32 _tizenrt_timerReset(_timerHandle xTimer,
						osdepTickType xBlockTime)
{
	DiagPrintf("%s %d\r\n", __func__, __LINE__);
	return 0;
}

void _tizenrt_acquire_wakelock(void)
{
}

void _tizenrt_release_wakelock(void)
{
}

void _tizenrt_wakelock_timeout(uint32_t timeout)
{
}

u8 _tizenrt_get_scheduler_state(void)
{
	DiagPrintf("%s %d\r\n", __func__, __LINE__);
	return 0;
}

const struct osdep_service_ops osdep_service = {
	_tizenrt_malloc,				  //rtw_vmalloc
	_tizenrt_zmalloc,				  //rtw_zvmalloc
	_tizenrt_mfree,					  //rtw_vmfree
	_tizenrt_malloc,				  //rtw_malloc
	_tizenrt_zmalloc,				  //rtw_zmalloc
	_tizenrt_mfree,					  //rtw_mfree
	_tizenrt_memcpy,				  //rtw_memcpy
	_tizenrt_memcmp,				  //rtw_memcmp
	_tizenrt_memset,				  //rtw_memset
	_tizenrt_init_sema,				  //rtw_init_sema
	_tizenrt_free_sema,				  //rtw_free_sema
	_tizenrt_up_sema,				  //rtw_up_sema
	_tizenrt_up_sema_from_isr,		  //rtw_up_sema_from_isr
	_tizenrt_down_sema,				  //rtw_down_sema
	_tizenrt_mutex_init,			  //rtw_mutex_init
	_tizenrt_mutex_free,			  //rtw_mutex_free
	_tizenrt_mutex_get,				  //rtw_mutex_get
	_tizenrt_mutex_get_timeout,		  //rtw_mutex_get_timeout
	_tizenrt_mutex_put,				  //rtw_mutex_put
	_tizenrt_enter_critical,		  //rtw_enter_critical
	_tizenrt_exit_critical,			  //rtw_exit_critical
	_tizenrt_enter_critical_from_isr, //rtw_enter_critical_from_isr
	_tizenrt_exit_critical_from_isr,  //rtw_exit_critical_from_isr
	NULL,							  //rtw_enter_critical_bh
	NULL,							  //rtw_exit_critical_bh
	_tizenrt_enter_critical_mutex,	//rtw_enter_critical_mutex
	_tizenrt_exit_critical_mutex,	 //rtw_exit_critical_mutex
	NULL,                            //rtw_suspend_task
	NULL,                            //rtw_resume_task
	_tizenrt_spinlock_init,		 //rtw_spinlock_init
	_tizenrt_spinlock_free,		 //rtw_spinlock_free
	_tizenrt_spinlock,			 //rtw_spin_lock
	_tizenrt_spinunlock,		 //rtw_spin_unlock
	_tizenrt_spinlock_irqsave,   //rtw_spinlock_irqsave
	_tizenrt_spinunlock_irqsave, //rtw_spinunlock_irqsave
	_tizenrt_init_xqueue,		 //rtw_init_xqueue
	_tizenrt_push_to_xqueue,	 //rtw_push_to_xqueue
	_tizenrt_pop_from_xqueue,	//rtw_pop_from_xqueue
	_tizenrt_deinit_xqueue,		 //rtw_deinit_xqueue
	_tizenrt_get_current_time,   //rtw_get_current_time
	_tizenrt_systime_to_ms,		 //rtw_systime_to_ms
	_tizenrt_systime_to_sec,	 //rtw_systime_to_sec
	_tizenrt_ms_to_systime,		 //rtw_ms_to_systime
	_tizenrt_sec_to_systime,	 //rtw_sec_to_systime
	_tizenrt_msleep_os,			 //rtw_msleep_os
	_tizenrt_usleep_os,			 //rtw_usleep_os
	_tizenrt_mdelay_os,			 //rtw_mdelay_os
	_tizenrt_udelay_os,			 //rtw_udelay_os
	_tizenrt_yield_os,			 //rtw_yield_os

	_tizenrt_ATOMIC_SET,		//ATOMIC_SET
	_tizenrt_ATOMIC_READ,		//ATOMIC_READ
	_tizenrt_ATOMIC_ADD,		//ATOMIC_ADD
	_tizenrt_ATOMIC_SUB,		//ATOMIC_SUB
	_tizenrt_ATOMIC_INC,		//ATOMIC_INC
	_tizenrt_ATOMIC_DEC,		//ATOMIC_DEC
	_tizenrt_ATOMIC_ADD_RETURN, //ATOMIC_ADD_RETURN
	_tizenrt_ATOMIC_SUB_RETURN, //ATOMIC_SUB_RETURN
	_tizenrt_ATOMIC_INC_RETURN, //ATOMIC_INC_RETURN
	_tizenrt_ATOMIC_DEC_RETURN, //ATOMIC_DEC_RETURN

	_tizenrt_modular64,		   //rtw_modular64
	_tizenrt_get_random_bytes, //rtw_get_random_bytes
	_tizenrt_GetFreeHeapSize,  //rtw_getFreeHeapSize

	_tizenrt_create_task, //rtw_create_task
	_tizenrt_delete_task, //rtw_delete_task
	_tizenrt_wake_task,   //rtw_wakeup_task
	NULL, 
	NULL,

	_tizenrt_thread_enter, //rtw_thread_enter
	_tizenrt_thread_exit,  //rtw_thread_exit

	_tizenrt_timerCreate,			   //rtw_timerCreate,
	_tizenrt_timerDelete,			   //rtw_timerDelete,
	_tizenrt_timerIsTimerActive,	   //rtw_timerIsTimerActive,
	_tizenrt_timerStop,				   //rtw_timerStop,
	_tizenrt_timerChangePeriod,		   //rtw_timerChangePeriod
	_tizenrt_timerGetID,			   //rtw_timerGetID
	_tizenrt_timerStart,			   //rtw_timerStart
	_tizenrt_timerStartFromISR,		   //rtw_timerStartFromISR
	_tizenrt_timerStopFromISR,		   //rtw_timerStopFromISR
	_tizenrt_timerResetFromISR,		   //rtw_timerResetFromISR
	_tizenrt_timerChangePeriodFromISR, //rtw_timerChangePeriodFromISR
	_tizenrt_timerReset,			   //rtw_timerReset

	_tizenrt_acquire_wakelock,   //rtw_acquire_wakelock
	_tizenrt_release_wakelock,   //rtw_release_wakelock
	_tizenrt_wakelock_timeout,   //rtw_wakelock_timeout
	_tizenrt_get_scheduler_state, //rtw_get_scheduler_state
	NULL,	// rtw_create_secure_context
	NULL,   //rtw_get_current_TaskHandle
};

static IRQ_FUN TizenUserIrqFunTable[MAX_PERIPHERAL_IRQ_NUM];
static int wrapper_IrqFun(int irq, FAR void *context, FAR void *arg)
{
	if(irq < AMEBAD_IRQ_FIRST){
		DiagPrintf("INT %d should not come here\r\n", irq);
		return OK;
	}
	__NVIC_ClearPendingIRQ(irq-AMEBAD_IRQ_FIRST);
	if (TizenUserIrqFunTable[irq-AMEBAD_IRQ_FIRST] != NULL) {
		TizenUserIrqFunTable[irq-AMEBAD_IRQ_FIRST]((VOID *)(arg));
	} else {
		DiagPrintf("INT_Entry Irq %d Fun Not Assign!!!!!", irq-AMEBAD_IRQ_FIRST);
	}
	return OK;
}

BOOL irq_register(IRQ_FUN IrqFun, IRQn_Type IrqNum, u32 Data,  u32 Priority) {
	if(IrqNum < 0){
		DiagPrintf("INT %d should not come here\r\n", IrqNum);
		return _TRUE;
	}
	Priority = (Priority >> IRQ_PRIORITY_SHIFT);
	if(IrqNum == WL_DMA_IRQ || IrqNum == WL_PROTOCOL_IRQ)
		Priority = 4;
	TizenUserIrqFunTable[IrqNum] = (IRQ_FUN)((u32)IrqFun | 0x1);
	Priority = (Priority << (8 - __NVIC_PRIO_BITS));
	irq_attach(IrqNum + AMEBAD_IRQ_FIRST, wrapper_IrqFun, (void *)Data);
	up_prioritize_irq(IrqNum + AMEBAD_IRQ_FIRST, Priority); //Need to fix, because it can't get the same result as __NVIC_SetPriority
	return _TRUE;
}

BOOL irq_unregister(IRQn_Type IrqNum)
{
	if(IrqNum < 0){
		DiagPrintf("INT %d should not come here\r\n", IrqNum);
		return _TRUE;
	}
	irq_detach(IrqNum);
	TizenUserIrqFunTable[IrqNum] = NULL;
	return _TRUE;
}

void irq_enable(IRQn_Type IrqNum)
{
	if(IrqNum < 0){
		DiagPrintf("INT %d should not come here\r\n", IrqNum);
		return;
	}
	up_enable_irq(IrqNum + AMEBAD_IRQ_FIRST);
}

void irq_disable(IRQn_Type IrqNum)
{
	if(IrqNum < 0){
		DiagPrintf("INT %d should not come here\r\n", IrqNum);
		return;
	}
	up_disable_irq(IrqNum + AMEBAD_IRQ_FIRST);
}

int __wrap_printf(const char *format, ...) {
    int ret = 0;
    va_list args;
    va_start(args, (const char *)format);
    ret = DiagPrintf((const char *)format, args);
    va_end(args);
    return ret;
}

void shell_switch_ipc_int(VOID *Data, u32 IrqStatus, u32 ChanNum)
{
	DiagPrintf("%s %d\r\n", __func__, __LINE__);
}

uint32_t * vTaskStackAddr(void)
{
	return NULL;
}

uint32_t vTaskStackSize(void)
{
	return 0;
}

void vPortEnterCritical(void)
{
	save_and_cli();
}
void vPortExitCritical(void)
{
	restore_flags();
}

