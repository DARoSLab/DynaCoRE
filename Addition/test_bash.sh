#! /bin/bash
echo "Hello"
PATH_PREFIX="/home"

if [[ "$OSTYPE" == "darwin"* ]]; then
    echo "MAC OS detected"
    PATH_PREFIX="/Users"
elif [[ "$OSTYPE" == "linux"*  ]]; then
    echo "LINUX OS detected
    PATH_PREFIX="/home"
else
    echo "Unsupported OS. Cannot run script""
fi
echo $PATH_PREFIX
