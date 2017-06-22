MAKE_FLAGS=-j5

clang-format-one:
	clang-format -i src/HelloWorld.h;

  #SOURCES := $(shell find $(SOURCEDIR) -name '*.c')

define format_folder
	cd $(1); \
	for file in *.h ; do \
		clang-format -i "$$file"; \
		echo $$file; \
	done
endef

clang-format:
	$(call format_folder,src) 

restore:
	cp src/HelloWorld.cc src/HelloWorld.h;
