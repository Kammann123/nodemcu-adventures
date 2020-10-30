/*******************************************************************************
  @file     queue
  @brief    Queue data structure handler
  @author   G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

#ifndef QUEUE_H_
#define QUEUE_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdio.h>
#include <stdbool.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

// Queue micro-framework feature!
// You can create a define with QUEUE_DEVELOPMENT_MODE to enable validation and
// verification of error in the functions while handling a Queue. Otherwise,
// verifications are not done to reduce code used.
//
// #define QUEUE_DEVELOPMENT_MODE

#define QUEUE_STANDARD_MAX_SIZE		30

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

// The implementation of the Queue uses one element to distinguish between
// a full and an empty queue, so the queueSize should be always one more
// than the actual maximum size desired.
typedef struct queue {
	uint32_t 	front;			// Next element to be out
	uint32_t	rear;			// Last element that entered the queue
	uint8_t*	buffer;			// Pointer to the array reserved in memory
	size_t 		queueSize;		// Amount of elements in the array (fixed)
	size_t		elementSize;	// Size in bytes of the element
} queue_t;

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Creates a Queue instance from the buffer and size specified by user
 * @param buffer		Pointer to the array reserved in memory
 * @param queueSize		Amount of elements in the array (fixed)
 * @param elementSize	Size in bytes of the element
 */
queue_t createQueue(void* buffer, size_t queueSize, size_t elementSize);

/**
 * @brief Returns whether the queue is empty or not
 * @param queue		Pointer to the Queue instance
 */
bool isEmpty(queue_t* queue);

/**
 * @brief Returns whether the queue is full or not
 * @param queue		Pointer to the Queue instance
 */
bool isFull(queue_t* queue);

/**
 * @brief Returns the current size of the queue
 * @param queue		Pointer to the Queue instance
 */
size_t size(queue_t* queue);

/**
 * @brief Returns the empty space for new elements in the queue
 * @param queue		Pointer to the Queue instance
 */
size_t emptySize(queue_t* queue);

/**
 * @brief Clear the Queue instance
 * @param queue		Pointer to the Queue instance
 */
void clear(queue_t* queue);

/**
 * @brief Push a new element to the queue, and returns true if succeed
 * 		  or false if the queue was full
 * @param queue		Pointer to the Queue instance
 * @param element	Pointer to the new element to be pushed
 */
bool push(queue_t* queue, void* element);

/**
 * @brief Pop the next element from the queue. If no elements left it returns
 * null, else the pointer to the next element, REMEMBER to copy the element!
 * ¡The persistance of the element is temporary! Use it at your own risk.
 */
void* pop(queue_t* queue);

/**
 * @brief Copies the given amount of elements from the queue to the destination 
 * buffer and pops them from the queue.
 * @param queue			Pointer to the Queue instance
 * @param destination	Pointer to the destination buffer
 * @param length		Number of elements to be copied
 */
void popMany(queue_t* queue, void* destination, size_t length);

/*******************************************************************************
 ******************************************************************************/

#endif
