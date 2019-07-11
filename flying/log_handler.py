from PIL import Image, ImageDraw, ImageFont

def write_probs_on_image(vv_prob, height_prob, offset_prob, view_prob, vertical_angle, image_array, img_path):
    # initialise the drawing context with
    # the image object as background
    img = Image.fromarray(image_array)
    draw = ImageDraw.Draw(img)
    # create font object with the font file and specify
    # desired size
    font = ImageFont.truetype('Roboto-Medium.ttf', size=15)
    
    # OFFSET
    (x, y) = (235, 450)
    color = 'rgb(0, 255, 255)' 
    draw.text((x, y), 'Offset: ', fill=color, font=font)
    
    (x, y) = (290, 450)
    color = 'rgb(255, 255, 0)' 
    draw.text((x, y), 'C', fill=color, font=font)
    draw.text((x+15, y), str("{0:.3f}".format(offset_prob[0])), fill=color, font=font)

    (x, y) = (355, 450)
    color = 'rgb(0, 255, 0)' #  color
    draw.text((x, y), 'M', fill=color, font=font)
    draw.text((x+15, y), str("{0:.3f}".format(offset_prob[1])), fill=color, font=font)

    (x, y) = (420, 450)
    color = 'rgb(255, 0, 255)' #  color
    draw.text((x, y), 'F', fill=color, font=font)
    draw.text((x+15, y), str("{0:.3f}".format(offset_prob[2])), fill=color, font=font)
    
    # VIEW
    (x, y) = (235, 420)
    color = 'rgb(0, 255, 255)' 
    draw.text((x, y), 'View: ', fill=color, font=font)
    
    (x, y) = (290, 420)
    color = 'rgb(255, 255, 0)' 
    draw.text((x, y), 'L', fill=color, font=font)
    draw.text((x+15, y), str("{0:.3f}".format(view_prob[0])), fill=color, font=font)

    (x, y) = (355, 420)
    color = 'rgb(0, 255, 0)' #  color
    draw.text((x, y), 'S', fill=color, font=font)
    draw.text((x+15, y), str("{0:.3f}".format(view_prob[1])), fill=color, font=font)

    (x, y) = (420, 420)
    color = 'rgb(255, 0, 255)' #  color
    draw.text((x, y), 'R', fill=color, font=font)
    draw.text((x+15, y), str("{0:.3f}".format(view_prob[2])), fill=color, font=font)
    
    # HEIGHT
    # starting position of the message
    (x, y) = (300, 300)
    color = 'rgb(0, 255, 255)' 
    draw.text((x, y), 'Altitude: ', fill=color, font=font)
    
    (x, y) = (300, 320)
    color = 'rgb(255, 255, 0)' 
    draw.text((x, y), 'H', fill=color, font=font)
    draw.text((x+20, y), str("{0:.3f}".format(height_prob[2])), fill=color, font=font)

    (x, y) = (300, 340)
    color = 'rgb(0, 255, 0)' #  color
    draw.text((x, y), 'M', fill=color, font=font)
    draw.text((x+20, y), str("{0:.3f}".format(height_prob[1])), fill=color, font=font)

    (x, y) = (300, 360)
    color = 'rgb(255, 0, 255)' #  color
    draw.text((x, y), 'L', fill=color, font=font)
    draw.text((x+20, y), str("{0:.3f}".format(height_prob[0])), fill=color, font=font)
    
    # VERTICAL_VIEW
    # starting position of the message
    (x, y) = (400, 300)
    color = 'rgb(0, 255, 255)' 
    draw.text((x, y), 'Vertical-View: ', fill=color, font=font)
    
    (x, y) = (400, 320)
    color = 'rgb(255, 255, 0)' 
    draw.text((x, y), 'U', fill=color, font=font)
    draw.text((x+20, y), str("{0:.3f}".format(vv_prob[2])), fill=color, font=font)

    (x, y) = (400, 340)
    color = 'rgb(0, 255, 0)' #  color
    draw.text((x, y), 'M', fill=color, font=font)
    draw.text((x+20, y), str("{0:.3f}".format(vv_prob[1])), fill=color, font=font)

    (x, y) = (400, 360)
    color = 'rgb(255, 0, 255)' #  color
    draw.text((x, y), 'D', fill=color, font=font)
    draw.text((x+20, y), str("{0:.3f}".format(vv_prob[0])), fill=color, font=font)

    # VERTICAL_VIEW
    # starting position of the message
    (x, y) = (400, 250)
    color = 'rgb(0, 255, 255)' 
    draw.text((x, y), 'Vertical-Angle: ', fill=color, font=font)
    color = 'rgb(255, 160, 16)' 
    draw.text((x+20, y+20), str("{0:.1f}".format(vertical_angle)), fill=color, font=font)
    
    
    # save the edited image
    img.save(img_path)
    
