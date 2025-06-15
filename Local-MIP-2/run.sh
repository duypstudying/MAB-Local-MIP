Lists=(MIPLIB-BP MIPLIB-IP MIPLIB-MBP MIPLIB-MIP)

for filename in Lists
do
  exec < ./benchmark/$filename.txt

  while read name

  do
    p="../instance/MIPLIB-BP/$name"
    ./Local-MIP --instance=$p --cutoff=60 -log=./$filename-60.csv
    ./Local-MIP --instance=$p --cutoff=10 -log=./$filename-10.csv
  done
done