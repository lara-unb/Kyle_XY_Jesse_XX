#! /bin/bash

if [ "$#" -eq 0 ]
then
	echo "Erro: esse script requer como argumento o numero do backup do dia atual"
	exit 1
fi

filename="programming.$(date +%F).$1.tar.bz2"

comando="tar -cjf $filename programming"
echo $comando
eval $comando
mv $filename backup 

filename="doc.$(date +%F).$1.tar.bz2"

comando="tar -cjf $filename doc"
echo $comando
eval $comando
mv $filename backup 

filename="temp.$(date +%F).$1.tar.bz2"

comando="tar -cjf $filename doc"
echo $comando
eval $comando
mv $filename backup 

