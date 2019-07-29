FROM jupyter/scipy-notebook

COPY worhp.deb .

ENV DEBIAN_FRONTEND=noninteractive
USER root
RUN apt-get update -y && apt-get install -y --install-recommends \
                                            libopenblas-base \
                                            libboost-python1.65.1 \
                                            libsuperlu5 \
                                            libatlas3-base \
                                            libgfortran3 \
                                            liblapack3 \
                                            libslicot0
RUN dpkg -i worhp.deb && apt-get install -f

RUN  apt-get install -y libblas3 libblas-dev liblapack3 liblapack-dev gfortran

COPY hsl /hsl
WORKDIR /hsl
RUN  ./configure --prefix=/usr LIBS="-llapack" --with-blas="-L/usr/lib -lblas" CXXFLAGS="-g -O2 -fopenmp" FCFLAGS="-g -O2 -fopenmp" CFLAGS="-g -O2 -fopenmp"
RUN make && make install
RUN ln -s /usr/lib/libcoinhsl.so /usr/lib/libhsl.so

RUN export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/hsl/lib

RUN conda install casadi

WORKDIR /home/jovyan
USER jovyan
CMD jupyter notebook