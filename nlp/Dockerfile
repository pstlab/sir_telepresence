FROM ubuntu:22.04
RUN apt-get update && apt-get install -y dialog apt-utils python3-pip git && pip3 install -U pip setuptools wheel && pip3 install -U spacy && python3 -m spacy download it_core_news_lg && python3 -c "import spacy; spacy.load('it_core_news_lg')" && pip3 install rasa gender-guesser
COPY . /home/nlp
RUN cd /home/nlp && rasa train
EXPOSE 5005/tcp
WORKDIR /home/nlp
CMD rasa run actions & rasa run -vv --enable-api