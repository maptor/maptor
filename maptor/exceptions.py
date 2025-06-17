import logging


logger = logging.getLogger(__name__)


class MAPTORBaseError(Exception):
    """
    Base class for all MAPTOR-specific errors.

    All MAPTOR exceptions inherit from this class, allowing users to catch
    any MAPTOR-specific error with a single except clause.

    Args:
        message: The error message describing what went wrong
        context: Optional additional context about where the error occurred
    """

    def __init__(self, message: str, context: str | None = None) -> None:
        self.message = message
        self.context = context

        logger.debug("MAPTOR exception: %s", self._format_message())
        super().__init__(self._format_message())

    def _format_message(self) -> str:
        if self.context:
            return f"{self.message} (Context: {self.context})"
        return self.message


class ConfigurationError(MAPTORBaseError):
    """
    Raised when there is an invalid or incomplete MAPTOR configuration.

    This exception indicates that the user has provided invalid parameters,
    missing required configuration, or conflicting settings that prevent
    the problem from being solved.

    Examples:
        - Invalid polynomial degrees
        - Missing dynamics or objective function
        - Incompatible constraint specifications
        - Invalid mesh configuration
    """

    pass


class DataIntegrityError(MAPTORBaseError):
    """
    Raised when internal data corruption or inconsistency is detected.

    This exception indicates an internal MAPTOR error where data structures
    have become corrupted or inconsistent. This typically represents a bug in
    MAPTOR rather than user error.

    Examples:
        - NaN or infinite values in computed results
        - Mismatched array dimensions in internal calculations
        - Corrupted solution data structures
    """

    pass


class SolutionExtractionError(MAPTORBaseError):
    """
    Raised when solution data cannot be extracted from the optimization result.

    This exception occurs when MAPTOR fails to process the raw solver output
    into the user-friendly Solution format, typically due to unexpected solver
    behavior or corrupted optimization results.
    """

    pass


class InterpolationError(MAPTORBaseError):
    """
    Raised when adaptive mesh interpolation operations fail.

    This exception occurs during adaptive mesh refinement when MAPTOR
    cannot successfully interpolate solution data between different mesh
    configurations, typically due to numerical issues or corrupted trajectory data.
    """

    pass
