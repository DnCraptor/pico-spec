/*
 * FreeRTOS V202212.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

/* Scheduler Related */
// Установите в 1 для использования планировщика с алгоритмом вытесняющей многозадачности (preemptive RTOS scheduler),
// или в 0 для использования кооперативной многозадачности (cooperative RTOS scheduler).
#define configUSE_PREEMPTION                    1
// Установите configUSE_TICKLESS_IDLE в 1 для использования режима пониженного потребления без тиков,
// или в 0 для сохранения всегда работающего прерывания тиков.
// Реализации режима без тиков с пониженным энергопотреблением не предоставляется для всех портов FreeRTOS.
#define configUSE_TICKLESS_IDLE                 0
// Установите в 1, если хотите использовать перехват состояния ожидания (idle hook), или в 0, если эта функция не нужна.
#define configUSE_IDLE_HOOK                     1
// Установите в 1, если хотите использовать перехват тиков, или 0, если это не нужно.
#define configUSE_TICK_HOOK                     1
// Здесь указывается частота в Гц, с которой тактируется внутреннее ядро микроконтроллера. На основе этой частоты аппаратный
// таймер будет генерировать прерывание тика. Обычно эта та же самая частота, на которой работает CPU. Это значение необходимо
// для корректного конфигурирования периферийных устройств таймера.
#define configCPU_RATE_HZ                       (366ul * 1000 * 1000)
// Частота прерываний тиков RTOS.
// Прерывание тика используется для измерения времени. Таким образом, чем выше частота тика, тем выше разрешающая способность
// отсчета времени. Однако высокая частота тиков также означает, что ядро RTOS будет больше тратить процессорного времени CPU,
// и эффективность работы приложения снизится. Все демо-приложения RTOS используют одинаковую частоту тиков 1000 Гц.
// Это значение использовалось для тестирования ядра RTOS, и большее значение обычно не требуется.
// У нескольких задач может быть настроен одинаковый приоритет. Планировщик RTOS будет распределять общее процессорное время
// между задачами с одинаковым приоритетом путем переключения между задачами на каждом тике RTOS. Таким образом, высокая
// частота тиков также дает эффект уменьшения слайса времени, выделенного для каждой задачи.
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )
// Количество приоритетов, доступных для задач приложения. Любое количество задач может могут использовать одинаковое значение
// приоритета. Для сопрограмм обработка приоритетов отдельная, см. configMAX_CO_ROUTINE_PRIORITIES.
// Каждый доступный приоритет потребляет RAM в ядре RTOS, поэтому параметр configMAX_PRIORITIES не следует устанавливать в
// значение больше, чем реально требует приложение.
#define configMAX_PRIORITIES                    32
// Размер стека, используемого для idle task. Обычно этот параметр не должен указываться меньше, чем значение в файл
// FreeRTOSConfig.h предоставленного демонстрационного приложения используемого вами порта FreeRTOS.
// Как и параметр размера стека для функций xTaskCreate() и xTaskCreateStatic(), размер стека в configMINIMAL_STACK_SIZE
// указывается в единицах слов, а не в байтах. Каждый элемент в стеке занимает 32-бита, поэтому размер стека 100 означает
// 400 байт (каждый 32-битный элемент стека занимает 4 байта).
#define configMINIMAL_STACK_SIZE                ( configSTACK_DEPTH_TYPE ) 256
// Максимально допустимая длина описательного имени, которое дается задаче при её создании. Длина указывается в количестве символов,
// включая символ-терминатор (нулевой байт).
#define configMAX_TASK_NAME_LEN                 16
// Установите в 1, если хотите подключить дополнительные поля структуры и функции, чтобы помочь с визуализацией выполнения задач
// и трассировкой.
#define configUSE_TRACE_FACILITY                1
// Установите configUSE_TRACE_FACILITY и configUSE_STATS_FORMATTING_FUNCTIONS в 1, чтобы подключить к сборке функции vTaskList()
// и vTaskGetRunTimeStats(). Либо установите в 0, чтобы опустить использование vTaskList() и vTaskGetRunTimeStates().
#define configUSE_STATS_FORMATTING_FUNCTIONS    1
// Время измерятся в "тиках". Количество срабатывания прерываний тиков с момента запуска ядра RTOS хранится в переменной TickType_t.
// Если определить configUSE_16_BIT_TICKS в 1, то тип TickType_t будет определен (через typedef) как 16-битное число без знака.
// Если определить configUSE_16_BIT_TICKS как 0, то TickType_t будет 32-битным числом без знака.
// Использование 16-битного типа значительно улучшает производительность на 8-битных и 16-битных архитектурах, но ограничивает
// при этом максимально доступный период времени в 65535 тиков. Таким образом, если предположить частоту тиков 250 Гц,
// то максимальная отслеживаемая задержка времени или блокировка при 16-битном счетчике составит 262 секунд (сравните с
// 17179869 секундами при 32-битном счетчике).
#define configUSE_16_BIT_TICKS                  0

// Этот параметр управляет поведением задач с приоритетом idle (самый низкий приоритет). Он дает эффект только если:
// • Используется планировщик с алгоритмом вытеснения (preemptive scheduler).
// • Приложение создает задачи, которые работают с приоритетом idle.
// Если параметр configUSE_TIME_SLICING установлен в 1 (или не определен), то задачи, которые используют одинаковый приоритет,
// будут переключаться на каждом слайсе времени (каждый тик). Если ни одна из задач не была вытеснена, то каждая задача с
// определенным приоритетом получит одинаковое количество процессорного времени, и если приоритет выше приоритета idle, то
// это действительно такой случай.
// Попадания в эту ситуацию можно избежать следующим образом:
// • Если это допустимо, использовать idle hook вместо отдельных задач с приоритетом idle.
// • Создание всех задач приложения с приоритетом выше приоритета idle.
// • Установка configIDLE_SHOULD_YIELD в 0.
// Установка configIDLE_SHOULD_YIELD в 0 предотвращает idle task от того, что она будет уступать процессорное время до
// окончания своего слайса времени. Это гарантирует, что все задачи с приоритетом idle получат одинаковое процессорное время
// (при условии, что ни одна из этих задач не была вытеснена) – однако ценой большего времени обработки, выделяемого для задачи idle.
#define configIDLE_SHOULD_YIELD                 1

/* Synchronization Related */
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_APPLICATION_TASK_TAG          0
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               8
#define configUSE_QUEUE_SETS                    1
#define configUSE_TIME_SLICING                  1
#define configUSE_NEWLIB_REENTRANT              0
#define configENABLE_BACKWARD_COMPATIBILITY     0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 5

/* System */
#define configSTACK_DEPTH_TYPE                  uint32_t
#define configMESSAGE_BUFFER_LENGTH_TYPE        size_t

/* Memory allocation related definitions. */
#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1
// 100 + 128
#define configTOTAL_HEAP_SIZE                   (228*1024)
#define configAPPLICATION_ALLOCATED_HEAP        0

/* Hook function related definitions. */
#define configCHECK_FOR_STACK_OVERFLOW          2
// Ядро использует вызов pvPortMalloc() для выделения памяти из кучи каждый раз, когда создается задача (task),
// очередь (queue) или семафор (semaphore). Официальный пакет загрузки FreeRTOS для этой цели включает в себя 4
// схемы примера выделения памяти, они реализованы соответственно в файлах heap_1.c, heap_2.c, heap_3.c, heap_4.c
// и heap_5.c. Параметр configUSE_MALLOC_FAILED_HOOK относится только к одной из этих схем, которая используется.
// Функция перехвата ошибки malloc() это hook-функция (или callback). Если она определена и сконфигурирована, то
// будет вызвана каждый раз, когда pvPortMalloc() вернет NULL. Значение NULL возвращается, если нет достаточного
// объема памяти в куче FreeRTOS, чтобы выполнить запрос приложения на выделение памяти.
// Если параметр configUSE_MALLOC_FAILED_HOOK установлен в 1, то приложение должно определить hook-функцию
// перехвата ошибки malloc(). Если configUSE_MALLOC_FAILED_HOOK установлен в 0, то функция перехвата ошибки
// malloc() не будет вызываться, даже если она была определена. Функция перехвата ошибки malloc() должна быть
// определена по следующему прототипу:
// void vApplicationMallocFailedHook( void );
#define configUSE_MALLOC_FAILED_HOOK            1
// Если оба параметра configUSE_TIMERS и configUSE_DAEMON_TASK_STARTUP_HOOK установлены в 1, то приложение должно
// определить hook-функцию точно с таким именем и прототипом, как показано ниже. Эта hook-функция будет вызвана
// точно только один раз, когда когда задача демона RTOS (также известная как timer service task) выполнится в
// первый раз. Любой инициализационный код приложения, которому нужен рабочий функционал RTOS, может быть помещен
// в эту hook-функцию.
// void vApplicationDaemonTaskStartupHook( void );
#define configUSE_DAEMON_TASK_STARTUP_HOOK      1

/* Run time and task stats gathering related definitions. */
#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                1
#define configUSE_STATS_FORMATTING_FUNCTIONS    0

/* Software timer related definitions. */
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               ( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            1024

/* Interrupt nesting behaviour configuration. */
/*
#define configKERNEL_INTERRUPT_PRIORITY         [dependent of processor]
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    [dependent on processor and application]
#define configMAX_API_CALL_INTERRUPT_PRIORITY   [dependent on processor and application]
*/

/* SMP port only */
#define configNUMBER_OF_CORES                   1
#define configTICK_CORE                         0
#define configRUN_MULTIPLE_PRIORITIES           0

/* RP2040 specific */
#define configSUPPORT_PICO_SYNC_INTEROP         1
#define configSUPPORT_PICO_TIME_INTEROP         1

#include <assert.h>
/* Define to trap errors during development. */
#define configASSERT(x)                         assert(x)

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xTaskGetIdleTaskHandle          1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xTimerPendFunctionCall          1
#define INCLUDE_xTaskAbortDelay                 1
#define INCLUDE_xTaskGetHandle                  1
#define INCLUDE_xTaskResumeFromISR              1
#define INCLUDE_xQueueGetMutexHolder            1

/* A header file that defines trace macro can be included here. */

/* SMP Related config. */
#define configUSE_PASSIVE_IDLE_HOOK             0
#define portSUPPORT_SMP                         1

#endif /* FREERTOS_CONFIG_H */

