.PHONY: hooks format

hooks:
	pre-commit install --hook-type pre-commit --hook-type commit-msg

format:
	ruff format .
	ruff check --fix .
	find chrono \( -path 'chrono/vendor' -o -path 'chrono/build' \) -prune -o \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \) -print0 | xargs -0 clang-format -i
	shfmt -i 4 -s -w .devcontainer/ docker/
