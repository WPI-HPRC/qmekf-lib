function R_EB = initialOrientationTRIAD(a_b, m_b, a_i, m_i)
% Returns a DCM Body -> Inertial
    % Normalize the input vectors
    a_b = a_b / norm(a_b);
    m_b = m_b / norm(m_b);
    a_i = a_i / norm(a_i);
    m_i = m_i / norm(m_i);

    % Calculate the reference vectors in inertial frame
    q_i = a_i;
    r_i = cross(a_i, m_i) / norm(cross(a_i, m_i));
    s_i = cross(q_i, r_i);

    M_i = [q_i, r_i, s_i];

    % Calculate the reference vectors in body frame
    q_b = a_b;
    r_b = cross(a_b, m_b) / norm(cross(a_b, m_b)); % Corrected to use m_b for body frame
    s_b = cross(q_b, r_b);

    M_b = [q_b, r_b, s_b];

    % Calculate the rotation matrix from body -> inertial
    R_EB = M_i * M_b';

    % Want R_EB.
end