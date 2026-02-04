package org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.base;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;


import org.firstinspires.ftc.robotcore.external.Consumer;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.autoCommands.LineToConstantAsync;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.autoCommands.LineToConstantAsyncAndConstraints;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.autoCommands.LineToConstantAsyncAndTime;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.autoCommands.LineToConstantSync;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.autoCommands.LineToLinearAsync;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.autoCommands.LineToLinearSync;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.autoCommands.LineToTangentAsync;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.autoCommands.LineToTangentSync;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.autoCommands.TangentToConstantAsync;
import org.firstinspires.ftc.teamcode.bravebots_decode.tasks.commandBased.commandTypes.Wait;
import org.firstinspires.ftc.teamcode.bravebots_decode.temu_pedro.Chassis;
import org.firstinspires.ftc.teamcode.bravebots_decode.utils.math.Pose;


import java.util.LinkedList;



public class Scheduler {

    Chassis f;


    boolean last= true, current= false, justDone= false;
    LinkedList<Task> queue;
    public Scheduler(){
        queue= new LinkedList<>();
    }
    public Scheduler(Chassis f){this.f =f; queue= new LinkedList<>(); }
    public boolean done(){
        return queue.isEmpty();
    }
    @Deprecated
    public int getQueueSize(){
        return queue.size();
    }

    public Scheduler addChassis(Chassis chassis){
        this.f= chassis;
        return this;
    }

    public Scheduler addTask(Task t){
        queue.addLast(t);
        return this;
    }
    public Scheduler addTask(InstantTask t){
        Task task= () -> {
            t.run();
            return true;
        };
        queue.addLast(task);
        return this;
    }
    public interface InstantTask{
        void run();
    }
    @Deprecated
    public boolean justDone(){
        return justDone;
    }
    public Scheduler waitSeconds(double sec){

        queue.addLast(new Wait(sec));
        return this;
    }
    @Deprecated
    public Scheduler waitSecondsFirst(double sec){
        queue.addFirst(new Wait(sec));
        return this;
    }

    public void removeAllTasks(){
        queue= new LinkedList<>();
    }
    @Deprecated
    public Scheduler addFirst(Task t){
        queue.addFirst(t);
        return this;
    }

    public void update(){

        if(done()) {
            current= true;
            justDone= !last;
            last= true;
            return;
        }
        current= false;
        last= false;


        Task t= queue.getFirst();
        boolean result= t.Run();
        if(result)
            queue.removeFirst();

    }
    public Scheduler tangentToConstantAsync(Pose p, double tValue)
    {
        if(f== null)
            throw new NullPointerException("baga Chassis in constructor");
        queue.addLast(new TangentToConstantAsync());
        return this;
    }
    public Scheduler lineToLinearSync(Pose p){
        if(f== null)
            throw new NullPointerException("baga Chassis in constructor");
        queue.addLast(new LineToLinearSync());
        return this;
    }
    public Scheduler lineToLinearAsync(Pose p){
        if(f== null)
            throw new NullPointerException("baga Chassis in constructor");
        queue.addLast(new LineToLinearAsync());
        return this;
    }

    public Scheduler lineToTangentialSync(Pose p){
        if(f== null)
            throw new NullPointerException("baga Chassis in constructor");
        queue.addLast(new LineToTangentSync());
        return this;
    }
    public Scheduler lineToTangentialAsync(Pose p){
        if(f== null)
            throw new NullPointerException("baga Chassis in constructor");
        queue.addLast(new
                LineToTangentAsync());
        return this;
    }public Scheduler lineToConstantSync(Pose p){
        if(f== null)
            throw new NullPointerException("baga Chassis in constructor");
        queue.addLast(new LineToConstantSync(this.f, p));
        return this;
    }public Scheduler lineToConstantAsync(Pose p){
        if(f== null)
            throw new NullPointerException("baga Chassis in constructor");
        queue.addLast(new LineToConstantAsync(this.f, p));
        return this;
    }
    public Scheduler lineToConstantAsync(Pose p, double time){
        if(f== null)
            throw new NullPointerException("baga Chassis in constructor");
        queue.addLast(new LineToConstantAsyncAndTime(this.f, p, time));
        return this;
    }
    public Scheduler lineToConstantAsync(Pose p, double dist, double rad){
        if(f== null)
            throw new NullPointerException("baga Chassis in constructor");
        queue.addLast(new LineToConstantAsyncAndConstraints(this.f, p, dist, rad));
        return this;
    }

}
