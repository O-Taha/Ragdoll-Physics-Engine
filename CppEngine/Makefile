.PHONY: build  docker-build docker-run

build:
	premake5 gmake2
	make -C build

docker-build:
	docker compose run --rm dev make build

docker-run:
	docker compose run --rm dev bash -c "make build"
	./bin/Debug/RagdollEngine