FROM ubuntu
RUN apt-get update && apt-get install -y apt-utils python3-pip haskell-platform git curl
RUN pip3 install -U pip setuptools wheel && pip3 install -U spacy && python3 -m spacy download it_core_news_lg && python3 -c "import spacy; spacy.load('it_core_news_lg')" && pip3 install rasa && cd home && curl -sSL https://get.haskellstack.org/ | sh && git clone https://github.com/facebook/duckling && cd duckling && stack init && stack build && cd..
COPY . nlp
RUN cd nlp && rasa train
EXPOSE 5005/tcp
CMD [ "rasa run actions & rasa run --enable-api" ]