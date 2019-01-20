package com.armabot.lidar.api;

import java.util.Optional;
import java.util.function.Consumer;

/**
 * Represents some kind of error.
 *
 * <p>
 * Each error has a code and some data. The string representation
 * is also suitable for easy reporting.
 * </p>
 *
 * <p>
 * Many API methods return {@code Optional<Error<?>>}. This should be
 * handled by {@link Optional#ifPresent(Consumer)} in most cases,
 * where you throw an exception better detailing the error for your
 * use case.
 * </p>
 */
public interface Error<T> {

    String code();

    T data();

    /**
     * Provides a reportable representation of the error.
     *
     * <p>Typically this will look like {@code [$code] $message}.</p>
     */
    @Override
    String toString();

}
