CARGO ?= cargo
BIN := sim
LOCAL_BIN := $(HOME)/.local/bin
DEBUG_BIN := $(CURDIR)/target/debug/$(BIN)
ARGS ?=

.PHONY: all build release rebuild run test test-watch lint fmt fmt-check check doc \
        link unlink install uninstall clean hooks format attach

all: build

# ------------------------------------ cli ----------------------------------- #

build:
	$(CARGO) build --workspace

release:
	$(CARGO) build --workspace --release

rebuild:
	$(CARGO) clean -p $(BIN)
	$(CARGO) build --workspace

run:
	$(CARGO) run --bin $(BIN) -- $(ARGS)

test:
	$(CARGO) test --workspace --all-targets

# needs `cargo install cargo-watch`
test-watch:
	$(CARGO) watch -x "test --workspace --all-targets"

lint:
	$(CARGO) clippy --workspace --all-targets --all-features -- -D warnings

fmt-check:
	$(CARGO) fmt --all --check

doc:
	$(CARGO) doc --workspace --no-deps --open

# everything CI checks, in CI order
check: fmt-check lint test

# --------------------------------- installing -------------------------------- #

# symlink the debug build onto PATH, so `make build` is enough to pick up changes
link: build
	mkdir -p $(LOCAL_BIN)
	ln -sf $(DEBUG_BIN) $(LOCAL_BIN)/$(BIN)
	@echo "linked $(LOCAL_BIN)/$(BIN) -> $(DEBUG_BIN)"

unlink:
	rm -f $(LOCAL_BIN)/$(BIN)

install:
	$(CARGO) install --path cli --locked

uninstall:
	$(CARGO) uninstall $(BIN)

clean:
	$(CARGO) clean

# -------------------------------- development ------------------------------- #

hooks:
	pre-commit install --hook-type pre-commit --hook-type commit-msg --hook-type pre-push

fmt format:
	$(CARGO) fmt --all
	ruff format .
	ruff check --fix .
	find chrono \( -path 'chrono/vendor' -o -path 'chrono/build' \) -prune -o \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \) -print0 | xargs -0 clang-format -i
	shfmt -i 4 -s -w .devcontainer/ docker/

attach:
	docker exec -it -e TERM=xterm-256color -e COLORTERM=truecolor simulations zsh
