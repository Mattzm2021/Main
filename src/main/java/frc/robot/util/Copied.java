package frc.robot.util;

import java.lang.annotation.*;

/**
 * this annotation represents the marked object is partly (or completely) copied from outside sources
 */
@Documented
@Retention(RetentionPolicy.SOURCE)
@Target({ElementType.METHOD, ElementType.TYPE})
public @interface Copied {

}
