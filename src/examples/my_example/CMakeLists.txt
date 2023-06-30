px4_add_module(
	MODULE examples__my_example
	MAIN my_example
	STACK_MAIN 2000
	SRCS
		my_example.cpp
	DEPENDS
)
