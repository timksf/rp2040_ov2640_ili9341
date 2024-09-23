add_compile_options(-Wall -Wextra)

add_library(tjpgd STATIC
    tjpgd.c
)

target_include_directories(tjpgd PUBLIC
	.
)