FROM ubuntu
RUN apt-get update && apt-get install -y apt-utils python3-pip haskell-platform git curl
RUN pip3 install -U pip setuptools wheel
RUN pip3 install -U spacy
RUN python3 -m spacy download it_core_news_lg
RUN python3 -c "import spacy; spacy.load('it_core_news_lg')"
RUN pip3 install rasa
RUN cd home
RUN curl -sSL https://get.haskellstack.org/ | sh
RUN git clone https://github.com/facebook/duckling
RUN cd duckling
RUN stack init
RUN stack build
RUN cd..
COPY . nlp
RUN cd nlp
RUN rasa train
EXPOSE 5005/tcp
CMD [ "rasa run actions & rasa run --enable-api" ]