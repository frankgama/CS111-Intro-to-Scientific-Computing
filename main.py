import matplotlib.pyplot as plt
import math
from matplotlib.animation import FuncAnimation

#frank gama, Final summer 2026
#“I confirm that I did not use codes from anyone else and that the work I submit is my own and my own only
# function that draws each frame of the animation
def animate(i):
    #initial conditions, time, time-step, end-time, positions, alpha and beta

    t = 0
    t_final = 50.0
    dt = 0.02
    r = .05

    #red    
    x_red = .75
    y_red = 5*r
    u_red = -.1
    v_red = .5
    x_old_r = x_red
    y_old_r = y_red
     

    #le bleu
    x_blue = .25
    y_blue = .275
    u_blue = .11
    v_blue = .2 
    x_old_b = x_blue
    y_old_b = y_blue
    #resistance, and friction
    alpha = .8
    beta = .98 
    dt_new = dt
    
    eps = 1e-12

    #loop computing Euler Steps
    while t < t_final:
        #cacluate positions
        x_red = x_old_r + (dt * u_red)
        y_red = y_old_r + (dt * v_red)
        x_blue = x_old_b + (dt * u_blue)
        y_blue = y_old_b + (dt * v_blue)

        #check distance between balls
        x_dist = x_red - x_blue
        y_dist = y_red - y_blue
        
       
        dist_balls = math.sqrt(x_dist**2 + y_dist**2)
        

        if dist_balls <= 2*r and dist_balls > eps:
            # unit normal from blue -> red with current position
            nx = x_dist / dist_balls
            ny = y_dist / dist_balls

            # relative normal speed 
            u_diff = u_red - u_blue
            v_diff = v_red - v_blue
            vrel = u_diff*nx + v_diff*ny  

            if vrel < 0.0:
                # 1) time to roll back from current predicted state to exact contact
                dt_back = (2*r - dist_balls) / (-vrel)
                # clamp for safety
                if dt_back < 0.0: dt_back = 0.0
                if dt_back > dt:  dt_back = dt

                #positions at contact
                x_rc = x_red  - u_red  * dt_back
                y_rc = y_red  - v_red  * dt_back
                x_bc = x_blue - u_blue * dt_back
                y_bc = y_blue - v_blue * dt_back

                # recompute normal at contact
                dxc = x_rc - x_bc
                dyc = y_rc - y_bc
                distc = math.hypot(dxc, dyc)
                if distc > eps:
                    nx = dxc / distc
                    ny = dyc / distc

                # swap normal components
                # tangential unit
                tx = -ny; ty = nx

                # projections at contact 
                n_red  = u_red*nx  + v_red*ny
                n_blue = u_blue*nx + v_blue*ny
                t_red  = u_red*tx  + v_red*ty
                t_blue = u_blue*tx + v_blue*ty

                # swapped normals
                n_red_new  = n_blue
                n_blue_new = n_red

                # post-collision velocities
                u_red  = n_red_new*nx  + t_red*tx
                v_red  = n_red_new*ny  + t_red*ty
                u_blue = n_blue_new*nx + t_blue*tx
                v_blue = n_blue_new*ny + t_blue*ty

                # advance from contact to end of this step with the new velocities
                rem = dt - dt_back
                x_red  = x_rc  + u_red  * rem
                y_red  = y_rc  + v_red  * rem
                x_blue = x_bc  + u_blue * rem
                y_blue = y_bc  + v_blue * rem

                # (tiny separation to avoid immediate retrigger)
                x_red  += 1e-9*nx;  y_red  += 1e-9*ny
                x_blue -= 1e-9*nx;  y_blue -= 1e-9*ny

                # record, update old, consume the whole dt, continue
                x_animation.append(x_red);    y_animation.append(y_red)
                x_animation_b.append(x_blue); y_animation_b.append(y_blue)
                x_old_r, y_old_r = x_red,  y_red
                x_old_b, y_old_b = x_blue, y_blue
                t += dt
                continue

        # #check for right bounds
        if x_red >= .95 or x_blue >= .95:
            #if only red is at bounds
            if x_blue <.95:
                dt_new = abs((.95 - x_red))/abs(u_red)
                x_red = x_old_r + (dt_new * u_red)
                y_red = y_old_r + (dt_new * v_red)
                x_blue = x_old_b + (dt_new * u_blue)
                y_blue = y_old_b + (dt_new * v_blue)
                x_animation.append(x_red)
                y_animation.append(y_red)
                x_animation_b.append(x_blue)
                y_animation_b.append(y_blue)
                x_old_r = x_red
                y_old_r = y_red
                x_old_b = x_blue
                y_old_b = y_blue
                u_red = -(alpha*u_red)
                v_red = beta*v_red
                t += dt_new
            #if only blue at bounds
            elif x_red < .95:
                dt_new = abs((.95 - x_blue))/abs(u_blue)
                x_red = x_old_r + (dt_new * u_red)
                y_red = y_old_r + (dt_new * v_red)
                x_blue = x_old_b + (dt_new * u_blue)
                y_blue = y_old_b + (dt_new * v_blue)
                x_animation.append(x_red)
                y_animation.append(y_red)
                x_animation_b.append(x_blue)
                y_animation_b.append(y_blue)
                x_old_r = x_red
                y_old_r = y_red
                x_old_b = x_blue
                y_old_b = y_blue
                u_blue = -(alpha*u_blue)
                v_blue = beta*v_blue
                t += dt_new
            #if both balls reach bounds at same time
            else:
                dt_new = abs((.95 - x_red))/abs(u_red)
                x_red = x_old_r + (dt_new * u_red)
                y_red = y_old_r + (dt_new * v_red)
                x_blue = x_old_b + (dt_new * u_blue)
                y_blue = y_old_b + (dt_new * v_blue)
                x_animation.append(x_red)
                y_animation.append(y_red)
                x_animation_b.append(x_blue)
                y_animation_b.append(y_blue)
                x_old_r = x_red
                y_old_r = y_red
                x_old_b = x_blue
                y_old_b = y_blue
                u_red = -(alpha*u_red)
                v_red = beta*v_red
                u_blue = -(alpha*u_blue)
                v_blue = beta*v_blue
                t += dt_new
        #check left bounds
        elif  x_red <= .05 or x_blue <=.05:
            #only red is at the left bound
            if x_blue > .05:
                dt_new = abs((.05-x_old_r)/u_red)
                x_red = x_old_r + (dt_new * u_red)
                y_red = y_old_r + (dt_new * v_red)
                x_blue = x_old_b + (dt_new * u_blue)
                y_blue = y_old_b + (dt_new * v_blue)
                x_animation.append(x_red)
                y_animation.append(y_red)
                x_animation_b.append(x_blue)
                y_animation_b.append(y_blue)
                x_old_r = x_red
                y_old_r = y_red
                x_old_b = x_blue
                y_old_b = y_blue
                u_red = -(alpha*u_red)
                v_red = beta*v_red
                t += dt_new
            #only blue is at the lefft bound
            elif x_red > .05:
                dt_new = abs((.05-x_old_b)/u_blue)
                x_red = x_old_r + (dt_new * u_red)
                y_red = y_old_r + (dt_new * v_red)
                x_blue = x_old_b + (dt_new * u_blue)
                y_blue = y_old_b + (dt_new * v_blue)
                x_animation.append(x_red)
                y_animation.append(y_red)
                x_animation_b.append(x_blue)
                y_animation_b.append(y_blue)
                x_old_r = x_red
                y_old_r = y_red
                x_old_b = x_blue
                y_old_b = y_blue
                u_blue = -(alpha*u_blue)
                v_blue = beta*v_blue
                t += dt_new
            #both at the bounds
            else:
                dt_new = abs((.05 - x_red))/abs(u_red)
                x_red = x_old_r + (dt_new * u_red)
                y_red = y_old_r + (dt_new * v_red)
                x_blue = x_old_b + (dt_new * u_blue)
                y_blue = y_old_b + (dt_new * v_blue)
                x_animation.append(x_red)
                y_animation.append(y_red)
                x_animation_b.append(x_blue)
                y_animation_b.append(y_blue)
                x_old_r = x_red
                y_old_r = y_red
                x_old_b = x_blue
                y_old_b = y_blue
                u_red = -(alpha*u_red)
                v_red = beta*v_red
                u_blue = -(alpha*u_blue)
                v_blue = beta*v_blue
                t += dt_new
        #check the top bounds
        elif y_red >= .95 or y_blue >= .95:
            #only red at the top
            if y_blue <.95:
                dt_new = abs((.95 - y_red))/abs(v_red)
                x_red = x_old_r + (dt_new * u_red)
                y_red = y_old_r + (dt_new * v_red)
                x_blue = x_old_b + (dt_new * u_blue)
                y_blue = y_old_b + (dt_new * v_blue)
                x_animation.append(x_red)
                y_animation.append(y_red)
                x_animation_b.append(x_blue)
                y_animation_b.append(y_blue)
                x_old_r = x_red
                y_old_r = y_red
                x_old_b = x_blue
                y_old_b = y_blue
                u_red = beta*u_red
                v_red = -alpha*v_red
                t += dt_new
            #only blue at the top
            elif y_red < .95:
                dt_new = abs((.95 - y_blue))/abs(v_blue)
                x_red = x_old_r + (dt_new * u_red)
                y_red = y_old_r + (dt_new * v_red)
                x_blue = x_old_b + (dt_new * u_blue)
                y_blue = y_old_b + (dt_new * v_blue)
                x_animation.append(x_red)
                y_animation.append(y_red)
                x_animation_b.append(x_blue)
                y_animation_b.append(y_blue)
                x_old_r = x_red
                y_old_r = y_red
                x_old_b = x_blue
                y_old_b = y_blue
                u_blue = beta*u_blue
                v_blue = -alpha*v_blue
                t += dt_new
            #both at the top
            else:
                dt_new = abs((.95 - y_red))/abs(v_red)
                x_red = x_old_r + (dt_new * u_red)
                y_red = y_old_r + (dt_new * v_red)
                x_blue = x_old_b + (dt_new * u_blue)
                y_blue = y_old_b + (dt_new * v_blue)
                x_animation.append(x_red)
                y_animation.append(y_red)
                x_animation_b.append(x_blue)
                y_animation_b.append(y_blue)
                x_old_r = x_red
                y_old_r = y_red
                x_old_b = x_blue
                y_old_b = y_blue
                u_red = beta*u_red
                v_red = -alpha*v_red
                u_blue = beta*u_blue
                v_blue = -alpha*v_blue
                t += dt_new
        #check the bottom bounds
        elif y_red <= .05 or y_blue <=.05:
            #only red at the top
            if y_blue >.05:
                dt_new = abs((.05 - y_red))/abs(v_red)
                x_red = x_old_r + (dt_new * u_red)
                y_red = y_old_r + (dt_new * v_red)
                x_blue = x_old_b + (dt_new * u_blue)
                y_blue = y_old_b + (dt_new * v_blue)
                x_animation.append(x_red)
                y_animation.append(y_red)
                x_animation_b.append(x_blue)
                y_animation_b.append(y_blue)
                x_old_r = x_red
                y_old_r = y_red
                x_old_b = x_blue
                y_old_b = y_blue
                u_red = beta*u_red
                v_red = -alpha*v_red
                t += dt_new
            #only blue at the top
            elif y_red > .05:
                dt_new = abs((.05 - y_blue))/abs(v_blue)
                x_red = x_old_r + (dt_new * u_red)
                y_red = y_old_r + (dt_new * v_red)
                x_blue = x_old_b + (dt_new * u_blue)
                y_blue = y_old_b + (dt_new * v_blue)
                x_animation.append(x_red)
                y_animation.append(y_red)
                x_animation_b.append(x_blue)
                y_animation_b.append(y_blue)
                x_old_r = x_red
                y_old_r = y_red
                x_old_b = x_blue
                y_old_b = y_blue
                u_blue = beta*u_blue
                v_blue = -alpha*v_blue
                t += dt_new
            #both at the top
            else:
                dt_new = abs((.05 - y_red))/abs(v_red)
                x_red = x_old_r + (dt_new * u_red)
                y_red = y_old_r + (dt_new * v_red)
                x_blue = x_old_b + (dt_new * u_blue)
                y_blue = y_old_b + (dt_new * v_blue)
                x_animation.append(x_red)
                y_animation.append(y_red)
                x_animation_b.append(x_blue)
                y_animation_b.append(y_blue)
                x_old_r = x_red
                y_old_r = y_red
                x_old_b = x_blue
                y_old_b = y_blue
                u_red = beta*u_red
                v_red = -alpha*v_red
                u_blue = beta*u_blue
                v_blue = -alpha*v_blue
                t += dt_new
        #balls no at bounds or touching
        else:
            x_animation.append(x_red)
            y_animation.append(y_red)
            x_animation_b.append(x_blue)
            y_animation_b.append(y_blue)
            x_old_r = x_red
            y_old_r = y_red
            x_old_b = x_blue    
            y_old_b = y_blue
            t += dt


    ax.clear()
    ax.set_aspect(1)
    circle = plt.Circle((x_animation[i], y_animation[i]), 0.05, color="red")
    blue = plt.Circle((x_animation_b[i], y_animation_b[i]), 0.05, color="blue")
    ax.add_artist(circle)
    ax.add_artist(blue) 
    ax.set_facecolor("forestgreen")
    ax.set_xlim([0, 1])
    ax.set_ylim([0, 1])


# create empty lists for the x and y coordinates
x_animation = []
y_animation = []
x_animation_b = []
y_animation_b = []


# create the figure and axes objects
fig, ax = plt.subplots()

# run the animation
ani = FuncAnimation(fig, animate, frames=200, interval=100, repeat=False)

plt.show()

