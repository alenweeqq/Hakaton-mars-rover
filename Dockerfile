FROM arena-base

LABEL org.opencontainers.image.version="0.16.0"

USER root
WORKDIR /submission
COPY . /submission/

RUN command -v g++ >/dev/null 2>&1 && \
    test -f "$(python -c 'import sysconfig; print(sysconfig.get_path("include"))')/Python.h" || \
    (apt-get update && apt-get install -y --no-install-recommends g++ python3-dev && \
     rm -rf /var/lib/apt/lists/*)
RUN python -c "import pybind11" >/dev/null 2>&1 || \
    python -m pip install --no-cache-dir "pybind11==3.1.0"
RUN python -m pip install --no-build-isolation --no-deps --force-reinstall /submission
RUN MARS_ROVER_REQUIRE_PLATFORM=1 python -m unittest discover -s /submission/tests -v

USER 10000:10000
CMD ["python", "/submission/train.py"]
