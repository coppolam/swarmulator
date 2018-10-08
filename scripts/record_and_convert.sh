# Bash script to record a simulation
# Requires the following package
#     - glc
#     - mencoder
  
cd ..

md=$(date +%Y-%m-%d-%T);
glc-capture -o rec_$md.glc  --fps=60 --disable-audio -s ./swarmulator $1

glc-play rec_$md.glc -o - -y 1 | avconv -i - -video_size 300x300 -pix_fmt yuv444p \
								 -threads auto -pix_fmt yuv444p -preset ultrafast -qp 0 -y rec_$md.mp4

# Move to movies folder
rm *.glc
mkdir -p movies
mv *.mp4 movies/

cd scripts
# glc-play rec_$md.glc -y 1 -o - | mencoder -demuxer y4m - \
    # -ovc lavc -lavcopts vcodec=mpeg4:vbitrate=10000 -o rec_$md.avi
    
