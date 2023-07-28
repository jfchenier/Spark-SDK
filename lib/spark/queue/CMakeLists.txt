add_library(queue "")

target_sources(queue
    PRIVATE
        circular_queue.c
        queue.c
    PUBLIC
        circular_queue.h
        queue.h
)
target_include_directories(queue PUBLIC ${CMAKE_CURRENT_LIST_DIR} ${SPARK_CIRC_QUEUE_OPT_PATH})
