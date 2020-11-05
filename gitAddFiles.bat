echo on
echo .
echo git add files to commmit ... 
echo allways the same files to add, so best put it in a batch file, 
echo needed since this somewhat strange ´staged-to-commit´ feature had been added
echo .
echo 

git add ST/STM32F767ZI_Nucleo/*
git add Micrium/*


git add .gitignore

git add gitAddFiles.bat
git add checkOriginMaster.bat


git status
