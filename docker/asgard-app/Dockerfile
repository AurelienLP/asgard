FROM navitia/asgard-build-deps:latest as builder

RUN git clone --depth=1 --branch test https://github.com/AurelienLP/asgard asgard

WORKDIR /asgard
RUN sed -i 's,git\@github.com:\([^/]*\)/\(.*\).git,https://github.com/\1/\2,' .gitmodules && git submodule update --init --recursive \
  && mkdir build && cd build \
  && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j$(nproc)

FROM debian:9
# We have to create a group and a user with the same gid and uid than the docker-data
RUN groupadd -g 1234 -r asgard-user && useradd --no-log-init -r -g asgard-user -u 1234 asgard-user
USER asgard-user
COPY --from=builder /asgard/build/asgard/asgard /usr/bin/asgard
COPY --from=builder /usr/lib/ /usr/lib/
COPY --from=builder /lib/ /lib/
EXPOSE 6000 8080
ENTRYPOINT ["/usr/bin/asgard"]
