#!/bin/bash

## Helper script to build SLAM-related containers
#
#

help()
{
    echo -e "Helper script to build SLAM-related containers

Usage: bash build_images.sh [OPTION]
[OPTION] are:
   --image=<image>                      Choose which image to build. Either 'fast-lio' or 'mavros'.
   --tag=<tag>                          Specify image tag (default: latest).
   --devel                              Adds '-dev' to the tag.
   --target=<target>                    Target companion PC. Either 'jetson' or 'raspberry'.
   --push                               After building, push the image to ghcr. User must be logged in.
   -h, --help                           Print this help."
   exit 0
}

TAG=latest
PUSH=false
DEV=false
while [ $# -gt 0 ]; do
  case "$1" in
    --tag=*)
      TAG="${1#*=}"
      ;;
    --push)
      PUSH=true
      ;;
    --devel)
      DEV=true
      ;;
    --target=*)
      TARGET="${1#*=}"
      ;;
    --image=*)
      IMAGE="${1#*=}"
      ;;
    --help|-h)
      help
      ;;
    *)
      help
      exit 1
  esac
  shift
done
echo "qui"



if [[ -z "$IMAGE" ]]; then
    echo "No image specified. Aborted."
    echo "-------------------------------------------"
    help
    exit 1
fi

if [ "$IMAGE" = "fast-lio" ]; then
    # Check if the user specified the companion PC
    if [[ -z "$TARGET" ]]; then
        echo "No target companion PC specified. Aborted."
        echo "-------------------------------------------"
        help
        exit 1
    fi
    if [ "$TARGET" = "jetson" ]; then
        if [ "$DEV" = true ]; then 
          TAG+="-dev"  # Append "-dev" to tag name
        fi
        docker build --build-arg RECLONE_REPOS=$(date +%s) -t ghcr.io/mavtech-srl/fast-lio-slam:$TAG -f Dockerfile .
        if [ "$PUSH" = true ]; then
            docker push ghcr.io/mavtech-srl/fast-lio-slam:$TAG
        fi
    elif [ "$TARGET" = "raspberry" ]; then
        TAG+="-rasp"
        if [ "$DEV" = true ]; then 
          TAG+="-dev"  # Append "-dev" to tag name
        fi
        docker build --build-arg RECLONE_REPOS=$(date +%s) -t ghcr.io/mavtech-srl/fast-lio-slam:$TAG -f Dockerfile.rasp .
        if [ "$PUSH" = true ]; then
            docker push ghcr.io/mavtech-srl/fast-lio-slam:$TAG
        fi
    else
        echo "The specified target is not valid. Aborted."
        echo "-------------------------------------------"
        help
        exit 1
    fi
elif [ "$IMAGE" = "mavros" ]; then
    docker build -t ghcr.io/mavtech-srl/mavros:$TAG -f Dockerfile.mavros .  
    if [ "$PUSH" = true ]; then
        docker push ghcr.io/mavtech-srl/mavros:$TAG
    fi
else
    echo "Specified image is not valid. Aborted."
    echo "-------------------------------------------"
    help
    exit 1
fi