/*
    FreeRTOS V7.0.2 - Copyright (C) 2011 Real Time Engineers Ltd.


    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

#ifndef SEMAPHORE_H
#define SEMAPHORE_H

#ifndef INC_FREERTOS_H
	#error "#include FreeRTOS.h" must appear in source files before "#include semphr.h"
#endif

#include "queue.h"

typedef xQueueHandle xSemaphoreHandle;

#define semBINARY_SEMAPHORE_QUEUE_LENGTH	( ( unsigned char ) 1U )
#define semSEMAPHORE_QUEUE_ITEM_LENGTH		( ( unsigned char ) 0U )
#define semGIVE_BLOCK_TIME					( ( portTickType ) 0U )


/**
 * semphr. h
 * <pre>vSemaphoreCreateBinary( xSemaphoreHandle xSemaphore )</pre>
 *
 * <i>Macro</i> that implements a semaphore by using the existing queue mechanism.
 * The queue length is 1 as this is a binary semaphore.  The data size is 0
 * as we don't want to actually store any data - we just want to know if the
 * queue is empty or full.
 *
 * This type of semaphore can be used for pure synchronisation between tasks or
 * between an interrupt and a task.  The semaphore need not be given back once
 * obtained, so one task/interrupt can continuously 'give' the semaphore while
 * another continuously 'takes' the semaphore.  For this reason this type of
 * semaphore does not use a priority inheritance mechanism.  For an alternative
 * that does use priority inheritance see xSemaphoreCreateMutex().
 *
 * @param xSemaphore Handle to the created semaphore.  Should be of type xSemaphoreHandle.
 *
 * Example usage:
 <pre>
 xSemaphoreHandle xSemaphore;

 void vATask( void * pvParameters )
 {
    // Semaphore cannot be used before a call to vSemaphoreCreateBinary ().
    // This is a macro so pass the variable in directly.
    vSemaphoreCreateBinary( xSemaphore );

    if( xSemaphore != NULL )
    {
        // The semaphore was created successfully.
        // The semaphore can now be used.  
    }
 }
 </pre>
 * \defgroup vSemaphoreCreateBinary vSemaphoreCreateBinary
 * \ingroup Semaphores
 */
#define vSemaphoreCreateBinary( xSemaphore )		{																									\
														( xSemaphore ) = xQueueCreate( ( unsigned portBASE_TYPE ) 1, semSEMAPHORE_QUEUE_ITEM_LENGTH );	\
														if( ( xSemaphore ) != NULL )																	\
														{																								\
															xSemaphoreGive( ( xSemaphore ) );															\
														}																								\
													}

/**
 * semphr. h
 * <pre>xSemaphoreTake( 
 *                   xSemaphoreHandle xSemaphore, 
 *                   portTickType xBlockTime 
 *               )</pre>
 *
 * <i>Macro</i> to obtain a semaphore.  The semaphore must have previously been
 * created with a call to vSemaphoreCreateBinary(), xSemaphoreCreateMutex() or
 * xSemaphoreCreateCounting().
 *
 * @param xSemaphore A handle to the semaphore being taken - obtained when
 * the semaphore was created.
 *
 * @param xBlockTime The time in ticks to wait for the semaphore to become
 * available.  The macro portTICK_RATE_MS can be used to convert this to a
 * real time.  A block time of zero can be used to poll the semaphore.  A block
 * time of portMAX_DELAY can be used to block indefinitely (provided
 * INCLUDE_vTaskSuspend is set to 1 in FreeRTOSConfig.h).
 *
 * @return pdTRUE if the semaphore was obtained.  pdFALSE
 * if xBlockTime expired without the semaphore becoming available.
 *
 * Example usage:
 <pre>
 xSemaphoreHandle xSemaphore = NULL;

 // A task that creates a semaphore.
 void vATask( void * pvParameters )
 {
    // Create the semaphore to guard a shared resource.
    vSemaphoreCreateBinary( xSemaphore );
 }

 // A task that uses the semaphore.
 void vAnotherTask( void * pvParameters )
 {
    // ... Do other things.

    if( xSemaphore != NULL )
    {
        // See if we can obtain the semaphore.  If the semaphore is not available
        // wait 10 ticks to see if it becomes free.	
        if( xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE )
        {
            // We were able to obtain the semaphore and can now access the
            // shared resource.

            // ...

            // We have finished accessing the shared resource.  Release the 
            // semaphore.
            xSemaphoreGive( xSemaphore );
        }
        else
        {
            // We could not obtain the semaphore and can therefore not access
            // the shared resource safely.
        }
    }
 }
 </pre>
 * \defgroup xSemaphoreTake xSemaphoreTake
 * \ingroup Semaphores
 */
#define xSemaphoreTake( xSemaphore, xBlockTime )		xQueueGenericReceive( ( xQueueHandle ) ( xSemaphore ), NULL, ( xBlockTime ), pdFALSE )

/**
 * semphr. h
 * xSemaphoreTakeRecursive( 
 *                          xSemaphoreHandle xMutex, 
 *                          portTickType xBlockTime 
 *                        )
 *
 * <i>Macro</i> to recursively obtain, or 'take', a mutex type semaphore.  
 * The mutex must have previously been created using a call to 
 * xSemaphoreCreateRecursiveMutex();
 * 
 * configUSE_RECURSIVE_MUTEXES must be set to 1 in FreeRTOSConfig.h for this
 * macro to be available.
 * 
 * This macro must not be used on mutexes created using xSemaphoreCreateMutex().
 *
 * A mutex used recursively can be 'taken' repeatedly by the owner. The mutex 
 * doesn't become available again until the owner has called 
 * xSemaphoreGiveRecursive() for each successful 'take' request.  For example, 
 * if a task successfully 'takes' the same mutex 5 times then the mutex will 
 * not be available to any other task until it has also  'given' the mutex back
 * exactly five times.
 *
 * @param xMutex A handle to the mutex being obtained.  This is the
 * handle returned by xSemaphoreCreateRecursiveMutex();
 *
 * @param xBlockTime The time in ticks to wait for the semaphore to become
 * available.  The macro portTICK_RATE_MS can be used to convert this to a
 * real time.  A block time of zero can be used to poll the semaphore.  If
 * the task already owns the semaphore then xSemaphoreTakeRecursive() will
 * return immediately no matter what the value of xBlockTime. 
 *
 * @return pdTRUE if the semaphore was obtained.  pdFALSE if xBlockTime
 * expired without the semaphore becoming available.
 *
 * Example usage:
 <pre>
 xSemaphoreHandle xMutex = NULL;

 // A task that creates a mutex.
 void vATask( void * pvParameters )
 {
    // Create the mutex to guard a shared resource.
    xMutex = xSemaphoreCreateRecursiveMutex();
 }

 // A task that uses the mutex.
 void vAnotherTask( void * pvParameters )
 {
    // ... Do other things.

    if( xMutex != NULL )
    {
        // See if we can obtain the mutex.  If the mutex is not available
        // wait 10 ticks to see if it becomes free.	
        if( xSemaphoreTakeRecursive( xSemaphore, ( portTickType ) 10 ) == pdTRUE )
        {
            // We were able to obtain the mutex and can now access the
            // shared resource.

            // ...
            // For some reason due to the nature of the code further calls to 
			// xSemaphoreTakeRecursive() are made on the same mutex.  In real
			// code these would not be just sequential calls as this would make
			// no sense.  Instead the calls are likely to be buried inside
			// a more complex call structure.
            xSemaphoreTakeRecursive( xMutex, ( portTickType ) 10 );
            xSemaphoreTakeRecursive( xMutex, ( portTickType ) 10 );

            // The mutex has now been 'taken' three times, so will not be 
			// available to another task until it has also been given back
			// three times.  Again it is unlikely that real code would have
			// these calls sequentially, but instead buried in a more complex
			// call structure.  This is just for illustrative purposes.
            xSemaphoreGiveRecursive( xMutex );
			xSemaphoreGiveRecursive( xMutex );
			xSemaphoreGiveRecursive( xMutex );

			// Now the mutex can be taken by other tasks.
        }
        else
        {
            // We could not obtain the mutex and can therefore not access
            // the shared resource safely.
        }
    }
 }
 </pre>
 * \defgroup xSemaphoreTakeRecursive xSemaphoreTakeRecursive
 * \ingroup Semaphores
 */
#define xSemaphoreTakeRecursive( xMutex, xBlockTime )	xQueueTakeMutexRecursive( ( xMutex ), ( xBlockTime ) )


/* 
 * xSemaphoreAltTake() is an alternative version of xSemaphoreTake().
 *
 * The source code that implements the alternative (Alt) API is much 
 * simpler	because it executes everything from within a critical section.  
 * This is	the approach taken by many other RTOSes, but FreeRTOS.org has the 
 * preferred fully featured API too.  The fully featured API has more 
 * complex	code that takes longer to execute, but makes much less use of 
 * critical sections.  Therefore the alternative API sacrifices interrupt 
 * responsiveness to gain execution speed, whereas the fully featured API
 * sacrifices execution speed to ensure better interrupt responsiveness.
 */
#define xSemaphoreAltTake( xSemaphore, xBlockTime )		xQueueAltGenericReceive( ( xQueueHandle ) ( xSemaphore ), NULL, ( xBlockTime ), pdFALSE )

/**
 * semphr. h
 * <pre>xSemaphoreGive( xSemaphoreHandle xSemaphore )</pre>
 *
 * <i>Macro</i> to release a semaphore.  The semaphore must have previously been
 * created with a call to vSemaphoreCreateBinary(), xSemaphoreCreateMutex() or
 * xSemaphoreCreateCounting(). and obtained using sSemaphoreTake().
 *
 * This macro must not be used from an ISR.  See xSemaphoreGiveFromISR () for
 * an alternative which can be used from an ISR.
 *
 * This macro must also not be used on semaphores created using 
 * xSemaphoreCreateRecursiveMutex().
 *
 * @param xSemaphore A handle to the semaphore being released.  This is the
 * handle returned when the semaphore was created.
 *
 * @return pdTRUE if the semaphore was released.  pdFALSE if an error occurred.
 * Semaphores are implemented using queues.  An error can occur if there is
 * no space on the queue to post a message - indicating that the 
 * semaphore was not first obtained correctly.
 *
 * Example usage:
 <pre>
 xSemaphoreHandle xSemaphore = NULL;

 void vATask( void * pvParameters )
 {
    // Create the semaphore to guard a shared resource.
    vSemaphoreCreateBinary( xSemaphore );

    if( xSemaphore != NULL )
    {
        if( xSemaphoreGive( xSemaphore ) != pdTRUE )
        {
            // We would expect this call to fail because we cannot give
            // a semaphore without first "taking" it!
        }

        // Obtain the semaphore - don't block if the semaphore is not
        // immediately available.
        if( xSemaphoreTake( xSemaphore, ( portTickType ) 0 ) )
        {
            // We now have the semaphore and can access the shared resource.

            // ...

            // We have finished accessing the shared resource so can free the
            // semaphore.
            if( xSemaphoreGive( xSemaphore ) != pdTRUE )
            {
                // We would not expect this call to fail because we must have
                // obtained the semaphore to get here.
            }
        }
    }
 }
 </pre>
 * \defgroup xSemaphoreGive xSemaphoreGive
 * \ingroup Semaphores
 */
#define xSemaphoreGive( xSemaphore )		xQueueGenericSend( ( xQueueHandle ) ( xSemaphore ), NULL, semGIVE_BLOCK_TIME, queueSEND_TO_BACK )

/**
 * semphr. h
 * <pre>xSemaphoreGiveRecursive( xSemaphoreHandle xMutex )</pre>
 *
 * <i>Macro</i> to recursively release, or 'give', a mutex type semaphore.
 * The mutex must have previously been created using a call to 
 * xSemaphoreCreateRecursiveMutex();
 * 
 * configUSE_RECURSIVE_MUTEXES must be set to 1 in FreeRTOSConfig.h for this
 * macro to be available.
 *
 * This macro must not be used on mutexes created using xSemaphoreCreateMutex().
 * 
 * A mutex used recursively can be 'taken' repeatedly by the owner. The mutex 
 * doesn't become available again until the owner has called 
 * xSemaphoreGiveRecursive() for each successful 'take' request.  For example, 
 * if a task successfully 'takes' the same mutex 5 times then the mutex will 
 * not be available to any other task until it has also  'given' the mutex back
 * exactly five times.
 *
 * @param xMutex A handle to the mutex being released, or 'given'.  This is the
 * handle returned by xSemaphoreCreateMutex();
 *
 * @return pdTRUE if the semaphore was given.
 *
 * Example usage:
 <pre>
 xSemaphoreHandle xMutex = NULL;

 // A task that creates a mutex.
 void vATask( void * pvParameters )
 {
    // Create the mutex to guard a shared resource.
    xMutex = xSemaphoreCreateRecursiveMutex();
 }

 // A task that uses the mutex.
 void vAnotherTask( void * pvParameters )
 {
    // ... Do other things.

    if( xMutex != NULL )
    {
        // See if we can obtain the mutex.  If the mutex is not available
        // wait 10 ticks to see if it becomes free.	
        if( xSemaphoreTakeRecursive( xMutex, ( portTickType ) 10 ) == pdTRUE )
        {
            // We were able to obtain the mutex and can now access the
            // shared resource.

            // ...
            // For some reason due to the nature of the code further calls to 
			// xSemaphoreTakeRecursive() are made on the same mutex.  In real
			// code these would not be just sequential calls as this would make
			// no sense.  Instead the calls are likely to be buried inside
			// a more complex call structure.
            xSemaphoreTakeRecursive( xMutex, ( portTickType ) 10 );
            xSemaphoreTakeRecursive( xMutex, ( portTickType ) 10 );

            // The mutex has now been 'taken' three times, so will not be 
			// available to another task until it has also been given back
			// three times.  Again it is unlikely that real code would have
			// these calls sequentially, it would be more likely that the calls
			// to xSemaphoreGiveRecursive() would be called as a call stack
			// unwound.  This is just for demonstrative purposes.
            xSemaphoreGiveRecursive( xMutex );
			xSemaphoreGiveRecursive( xMutex );
			xSemaphoreGiveRecursive( xMutex );

			// Now the mutex can be taken by other tasks.
        }
        else
        {
            // We could not obtain the mutex and can therefore not access
            // the shared resource safely.
        }
    }
 }
 </pre>
 * \defgroup xSemaphoreGiveRecursive xSemaphoreGiveRecursive
 * \ingroup Semaphores
 */
#define xSemaphoreGiveRecursive( xMutex )	xQueueGiveMutexRecursive( ( xMutex ) )

/* 
 * xSemaphoreAltGive() is an alternative version of xSemaphoreGive().
 *
 * The source code that implements the alternative (Alt) API is much 
 * simpler	because it executes everything from within a critical section.  
 * This is	the approach taken by many other RTOSes, but FreeRTOS.org has the 
 * preferred fully featured API too.  The fully featured API has more 
 * complex	code that takes longer to execute, but makes much less use of 
 * critical sections.  Therefore the alternative API sacrifices interrupt 
 * responsiveness to gain execution speed, whereas the fully featured API
 * sacrifices execution speed to ensure better interrupt responsiveness.
 */
#define xSemaphoreAltGive( xSemaphore )		xQueueAltGenericSend( ( xQueueHandle ) ( xSemaphore ), NULL, semGIVE_BLOCK_TIME, queueSEND_TO_BACK )

/**
 * semphr. h
 * <pre>
 xSemaphoreGiveFromISR( 
                          xSemaphoreHandle xSemaphore, 
                          signed portBASE_TYPE *pxHigherPriorityTaskWoken
                      )</pre>
 *
 * <i>Macro</i> to  release a semaphore.  The semaphore must have previously been
 * created with a call to vSemaphoreCreateBinary() or xSemaphoreCreateCounting().
 *
 * Mutex type semaphores (those created using a call to xSemaphoreCreateMutex())
 * must not be used with this macro.
 *
 * This macro can be used from an ISR.
 *
 * @param xSemaphore A handle to the semaphore being released.  This is the
 * handle returned when the semaphore was created.
 *
 * @param pxHigherPriorityTaskWoken xSemaphoreGiveFromISR() will set
 * *pxHigherPriorityTaskWoken to pdTRUE if giving the semaphore caused a task
 * to unblock, and the unblocked task has a priority higher than the currently
 * running task.  If xSemaphoreGiveFromISR() sets this value to pdTRUE then
 * a context switch should be requested before the interrupt is exited.
 *
 * @return pdTRUE if the semaphore was successfully given, otherwise errQUEUE_FULL.
 *
 * Example usage:
 <pre>
 \#define LONG_TIME 0xffff
 \#define TICKS_TO_WAIT	10
 xSemaphoreHandle xSemaphore = NULL;

 // Repetitive task.
 void vATask( void * pvParameters )
 {
    for( ;; )
    {
        // We want this task to run every 10 ticks of a timer.  The semaphore 
        // was created before this task was started.

        // Block waiting for the semaphore to become available.
        if( xSemaphoreTake( xSemaphore, LONG_TIME ) == pdTRUE )
        {
            // It is time to execute.

            // ...

            // We have finished our task.  Return to the top of the loop where
            // we will block on the semaphore until it is time to execute 
            // again.  Note when using the semaphore for synchronisation with an
			// ISR in this manner there is no need to 'give' the semaphore back.
        }
    }
 }

 // Timer ISR
 void vTimerISR( void * pvParameters )
 {
 static unsigned char ucLocalTickCount = 0;
 static signed portBASE_TYPE xHigherPriorityTaskWoken;

    // A timer tick has occurred.

    // ... Do other time functions.

    // Is it time for vATask () to run?
	xHigherPriorityTaskWoken = pdFALSE;
    ucLocalTickCount++;
    if( ucLocalTickCount >= TICKS_TO_WAIT )
    {
        // Unblock the task by releasing the semaphore.
        xSemaphoreGiveFromISR( xSemaphore, &xHigherPriorityTaskWoken );

        // Reset the count so we release the semaphore again in 10 ticks time.
        ucLocalTickCount = 0;
    }

    if( xHigherPriorityTaskWoken != pdFALSE )
    {
        // We can force a context switch here.  Context switching from an
        // ISR uses port specific syntax.  Check the demo task for your port
        // to find the syntax required.
    }
 }
 </pre>
 * \defgroup xSemaphoreGiveFromISR xSemaphoreGiveFromISR
 * \ingroup Semaphores
 */
#define xSemaphoreGiveFromISR( xSemaphore, pxHigherPriorityTaskWoken )			xQueueGenericSendFromISR( ( xQueueHandle ) ( xSemaphore ), NULL, ( pxHigherPriorityTaskWoken ), queueSEND_TO_BACK )

/**
 * semphr. h
 * <pre>xSemaphoreHandle xSemaphoreCreateMutex( void )</pre>
 *
 * <i>Macro</i> that implements a mutex semaphore by using the existing queue 
 * mechanism.
 *
 * Mutexes created using this macro can be accessed using the xSemaphoreTake()
 * and xSemaphoreGive() macros.  The xSemaphoreTakeRecursive() and 
 * xSemaphoreGiveRecursive() macros should not be used.
 * 
 * This type of semaphore uses a priority inheritance mechanism so a task 
 * 'taking' a semaphore MUST ALWAYS 'give' the semaphore back once the 
 * semaphore it is no longer required.  
 *
 * Mutex type semaphores cannot be used from within interrupt service routines.  
 *
 * See vSemaphoreCreateBinary() for an alternative implementation that can be 
 * used for pure synchronisation (where one task or interrupt always 'gives' the 
 * semaphore and another always 'takes' the semaphore) and from within interrupt 
 * service routines.
 *
 * @return xSemaphore Handle to the created mutex semaphore.  Should be of type 
 *		xSemaphoreHandle.
 *
 * Example usage:
 <pre>
 xSemaphoreHandle xSemaphore;

 void vATask( void * pvParameters )
 {
    // Semaphore cannot be used before a call to xSemaphoreCreateMutex().
    // This is a macro so pass the variable in directly.
    xSemaphore = xSemaphoreCreateMutex();

    if( xSemaphore != NULL )
    {
        // The semaphore was created successfully.
        // The semaphore can now be used.  
    }
 }
 </pre>
 * \defgroup vSemaphoreCreateMutex vSemaphoreCreateMutex
 * \ingroup Semaphores
 */
#define xSemaphoreCreateMutex() xQueueCreateMutex()


/**
 * semphr. h
 * <pre>xSemaphoreHandle xSemaphoreCreateRecursiveMutex( void )</pre>
 *
 * <i>Macro</i> that implements a recursive mutex by using the existing queue 
 * mechanism.
 *
 * Mutexes created using this macro can be accessed using the 
 * xSemaphoreTakeRecursive() and xSemaphoreGiveRecursive() macros.  The 
 * xSemaphoreTake() and xSemaphoreGive() macros should not be used.
 *
 * A mutex used recursively can be 'taken' repeatedly by the owner. The mutex 
 * doesn't become available again until the owner has called 
 * xSemaphoreGiveRecursive() for each successful 'take' request.  For example, 
 * if a task successfully 'takes' the same mutex 5 times then the mutex will 
 * not be available to any other task until it has also  'given' the mutex back
 * exactly five times.
 * 
 * This type of semaphore uses a priority inheritance mechanism so a task 
 * 'taking' a semaphore MUST ALWAYS 'give' the semaphore back once the 
 * semaphore it is no longer required.  
 *
 * Mutex type semaphores cannot be used from within interrupt service routines.  
 *
 * See vSemaphoreCreateBinary() for an alternative implementation that can be 
 * used for pure synchronisation (where one task or interrupt always 'gives' the 
 * semaphore and another always 'takes' the semaphore) and from within interrupt 
 * service routines.
 *
 * @return xSemaphore Handle to the created mutex semaphore.  Should be of type 
 *		xSemaphoreHandle.
 *
 * Example usage:
 <pre>
 xSemaphoreHandle xSemaphore;

 void vATask( void * pvParameters )
 {
    // Semaphore cannot be used before a call to xSemaphoreCreateMutex().
    // This is a macro so pass the variable in directly.
    xSemaphore = xSemaphoreCreateRecursiveMutex();

    if( xSemaphore != NULL )
    {
        // The semaphore was created successfully.
        // The semaphore can now be used.  
    }
 }
 </pre>
 * \defgroup vSemaphoreCreateMutex vSemaphoreCreateMutex
 * \ingroup Semaphores
 */
#define xSemaphoreCreateRecursiveMutex() xQueueCreateMutex()

/**
 * semphr. h
 * <pre>xSemaphoreHandle xSemaphoreCreateCounting( unsigned portBASE_TYPE uxMaxCount, unsigned portBASE_TYPE uxInitialCount )</pre>
 *
 * <i>Macro</i> that creates a counting semaphore by using the existing 
 * queue mechanism.  
 *
 * Counting semaphores are typically used for two things:
 *
 * 1) Counting events.  
 *
 *    In this usage scenario an event handler will 'give' a semaphore each time
 *    an event occurs (incrementing the semaphore count value), and a handler 
 *    task will 'take' a semaphore each time it processes an event 
 *    (decrementing the semaphore count value).  The count value is therefore 
 *    the difference between the number of events that have occurred and the 
 *    number that have been processed.  In this case it is desirable for the 
 *    initial count value to be zero.
 *
 * 2) Resource management.
 *
 *    In this usage scenario the count value indicates the number of resources
 *    available.  To obtain control of a resource a task must first obtain a 
 *    semaphore - decrementing the semaphore count value.  When the count value
 *    reaches zero there are no free resources.  When a task finishes with the
 *    resource it 'gives' the semaphore back - incrementing the semaphore count
 *    value.  In this case it is desirable for the initial count value to be
 *    equal to the maximum count value, indicating that all resources are free.
 *
 * @param uxMaxCount The maximum count value that can be reached.  When the 
 *        semaphore reaches this value it can no longer be 'given'.
 *
 * @param uxInitialCount The count value assigned to the semaphore when it is
 *        created.
 *
 * @return Handle to the created semaphore.  Null if the semaphore could not be
 *         created.
 * 
 * Example usage:
 <pre>
 xSemaphoreHandle xSemaphore;

 void vATask( void * pvParameters )
 {
 xSemaphoreHandle xSemaphore = NULL;

    // Semaphore cannot be used before a call to xSemaphoreCreateCounting().
    // The max value to which the semaphore can count should be 10, and the
    // initial value assigned to the count should be 0.
    xSemaphore = xSemaphoreCreateCounting( 10, 0 );

    if( xSemaphore != NULL )
    {
        // The semaphore was created successfully.
        // The semaphore can now be used.  
    }
 }
 </pre>
 * \defgroup xSemaphoreCreateCounting xSemaphoreCreateCounting
 * \ingroup Semaphores
 */
#define xSemaphoreCreateCounting( uxMaxCount, uxInitialCount ) xQueueCreateCountingSemaphore( ( uxMaxCount ), ( uxInitialCount ) )

/**
 * semphr. h
 * <pre>void vSemaphoreDelete( xSemaphoreHandle xSemaphore );</pre>
 *
 * Delete a semaphore.  This function must be used with care.  For example,
 * do not delete a mutex type semaphore if the mutex is held by a task.
 *
 * @param xSemaphore A handle to the semaphore to be deleted.
 *
 * \page vSemaphoreDelete vSemaphoreDelete
 * \ingroup Semaphores
 */
#define vSemaphoreDelete( xSemaphore ) vQueueDelete( ( xQueueHandle ) xSemaphore )

#endif /* SEMAPHORE_H */


