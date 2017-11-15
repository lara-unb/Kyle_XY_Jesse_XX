#! /bin/bash

if [ "$#" -eq 0 ]
then
	echo "Erro: esse script requer como argumento o nome do modulo"
	exit 1
fi

modulename="$1"

cp module-template.c $modulename.c
cp module-template.h $modulename.h

#command="sed 's/module/teste/g' $modulename.c > $modulename-temp.c"
#echo $command
#exec $command

sed -i 's/module/'$modulename'/g' $modulename.c
sed -i 's/DATADECRIACAO/'$(date +%d-%m-%Y)'/g' $modulename.c

sed -i 's/module/'$modulename'/g' $modulename.h
sed -i 's/DATADECRIACAO/'$(date +%d-%m-%Y)'/g' $modulename.h
sed -i 's/MODULE/'$(echo $1 | tr '[:lower:]' '[:upper:]')'/g' $modulename.h

if [ "$2" != '' ]
then
	mv -f $modulename.* $2
fi
echo "Modulo $modulename criado com sucesso."

