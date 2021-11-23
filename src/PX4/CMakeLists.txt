px4_add_module(
	MODULE drivers__nsl_adu
	MAIN nsl_adu
	COMPILE_FLAGS
	SRCS
		nsl_adu.cpp
	DEPENDS
		mathlib
	)
